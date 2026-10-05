//! Nodal device evaluation, self-heating updates and voltage limiting.

use super::residual::emit_sparse_nv_dot;
use crate::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use crate::codegen::rust_emitter::helpers::{
    body_effect_mosfets, emit_thermal_tj_advance, pentode_dispatch,
};
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Emit per-sample device self-heating thermal updates (BJT, diode, triode).
    ///
    /// Shared by the Schur and full-LU nodal `process_sample` emitters — both
    /// call this at 4-space indent after `i_nl` and `v` are final for the
    /// sample, so the dissipated power P uses converged values.
    ///
    /// Thermal ODE: `CTH·dTj/dt = P - (Tj - TAMB)/RTH`. For fixed P this is
    /// linear in Tj, so the per-sample step is the EXACT exponential update
    /// toward the steady state `Tss = TAMB + P·RTH`:
    ///
    /// ```ignore
    /// Tj += (Tss - Tj) * (1.0 - (-dt/tau).exp());   // tau = RTH*CTH
    /// ```
    ///
    /// which is unconditionally stable for any dt/τ (the previous forward
    /// Euler step diverged when dt > 2τ). When `CTH ≤ 0` the thermal pole is
    /// infinitely fast — the quasi-static form `Tj = Tss` is emitted at
    /// codegen time instead (no division by CTH).
    ///
    /// `dt` is the INTERNAL sample period: this code runs inside the
    /// per-internal-sample body, which executes OVERSAMPLING_FACTOR× per base
    /// sample. The rate is read from `state.current_sample_rate` (HOST-rate
    /// semantics, kept live by `set_sample_rate`) × OVERSAMPLING_FACTOR, NOT
    /// the baked SAMPLE_RATE/INTERNAL_SAMPLE_RATE consts — mirrors the DK
    /// path's `emit_thermal_tj_advance`. A baked dt makes the thermal time
    /// constant scale with fs_host/fs_codegen after `set_sample_rate` (e.g.
    /// 2× too-fast heating at 96 kHz on a 48 kHz build). The
    /// `needs_current_sr` gate includes thermal devices so the field always
    /// exists here.
    ///
    /// Temperature scaling after the Tj step:
    /// - BJT: `VT(T) = k/q·Tj`, `IS(T) = IS·(Tj/TAMB)^XTI·exp((EG/vt_nom)·(1-TAMB/Tj))`
    ///   (SPICE3f5 BJT law — no ideality division).
    /// - Diode: `N·VT(T)` scales linearly; `IS(T)` divides both exponents by
    ///   the ideality factor N (SPICE3f5 diode law:
    ///   `IS(T) = IS·(Tj/TAMB)^(XTI/N)·exp((EG/(N·vt_nom))·(1-TAMB/Tj))`).
    ///   `IS_NOM` and `N_VT_NOM` are the card's values at TAMB
    ///   (`resolve_diode_params` scales them from TNOM), so the law composes.
    ///   N is recovered at codegen time as `dp.n_vt / Vt(TAMB)`, the thermal
    ///   voltage `resolve_diode_params` used to build `n_vt = N·Vt(TAMB)`.
    /// - Triode: Tj only — the Koren coefficients are untouched; the drift
    ///   rides the `VBIAS_ALPHA·(Tj-TAMB)` Vgk shift at the NR call sites.
    pub(super) fn emit_self_heating_thermal_updates(code: &mut String, ir: &CircuitIR) {
        // The Tj advance inner block is the twin-shared source of truth in
        // `super::super::helpers::emit_thermal_tj_advance` (byte-identical to the DK
        // path, enforced by `thermal_tj_advance_dk_nodal_string_identity`).
        // It expects `p` (dissipated power, W) in scope and leaves the
        // [200,500] K clamped Tj in `state.device_{dev_num}_tj`.
        let tj_step = emit_thermal_tj_advance;
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                // The `device_type == Bjt` guard (not BjtForwardActive) is
                // belt-and-braces against slot aliasing: this arm reads the
                // 2D slot pair (i_nl[s], i_nl[s+1]); on a 1D FA-reduced slot
                // s+1 would be the NEXT device's slot (or out of bounds).
                // detect_forward_active_bjts excludes self-heating BJTs from
                // FA reduction, so this guard should never fire — but if that
                // gating ever regresses, aliasing stays structurally
                // impossible here.
                DeviceParams::Bjt(bp)
                    if bp.has_self_heating() && slot.device_type == DeviceType::Bjt =>
                {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "    {{ // BJT {dev_num} self-heating thermal update\n\
                         \x20       let ic = i_nl[{s}];\n\
                         \x20       let ib = i_nl[{s1}];\n\
                         \x20       let mut vbe_sum = 0.0f64;\n\
                         \x20       let mut vbc_sum = 0.0f64;\n\
                         \x20       for j in 0..N {{ vbe_sum += N_V[{s}][j] * v[j]; }}\n\
                         \x20       for j in 0..N {{ vbc_sum += N_V[{s1}][j] * v[j]; }}\n\
                         \x20       let vce = vbe_sum - vbc_sum;\n\
                         \x20       let p = vce * ic + vbe_sum * ib;\n"
                    ));
                    code.push_str(&tj_step(dev_num, bp.cth));
                    code.push_str(&format!(
                        "\x20       state.device_{dev_num}_vt = BOLTZMANN_Q * state.device_{dev_num}_tj;\n\
                         \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                         \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                         \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI)\n\
                         \x20           * fast_exp((DEVICE_{dev_num}_EG / vt_nom) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                         \x20       // Beta temperature dependence (SPICE XTB) — MUST stay in step with\n\
                         \x20       // the DK twin in dk_emitter.rs. XTB defaults to 0.0, so `powf`\n\
                         \x20       // returns exactly 1.0 and this is inert on cards that omit it.\n\
                         \x20       state.device_{dev_num}_bf = DEVICE_{dev_num}_BETA_F * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                         \x20       state.device_{dev_num}_br = DEVICE_{dev_num}_BETA_R * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                         \x20   }}\n"
                    ));
                }
                DeviceParams::Diode(dp) if dp.has_self_heating() => {
                    let s = slot.start_idx;
                    // SPICE3f5 diode law divides both IS(T) exponents by the
                    // ideality factor N. n_vt was built as N·Vt(TAMB).
                    let n_ideality = dp.n_vt
                        / (melange_primitives::VT_ROOM * (dp.tamb / melange_primitives::T_NOM));
                    code.push_str(&format!(
                        "    {{ // Diode {dev_num} self-heating thermal update\n\
                         \x20       let id = i_nl[{s}];\n\
                         \x20       let mut vd_sum = 0.0f64;\n\
                         \x20       for j in 0..N {{ vd_sum += N_V[{s}][j] * v[j]; }}\n\
                         \x20       let p = vd_sum * id;\n"
                    ));
                    code.push_str(&tj_step(dev_num, dp.cth));
                    code.push_str(&format!(
                        "\x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_n_vt = DEVICE_{dev_num}_N_VT_NOM * t_ratio;\n\
                         \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                         \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI / {n_ideality:.17e})\n\
                         \x20           * fast_exp((DEVICE_{dev_num}_EG / ({n_ideality:.17e} * vt_nom)) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                         \x20   }}\n"
                    ));
                }
                DeviceParams::Tube(tp) if tp.has_self_heating() => {
                    // Pdiss = Ip·Vpk + Ig·Vgk using converged i_nl and the
                    // same N_V·v contraction the BJT path uses.
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "    {{ // Triode {dev_num} self-heating thermal update\n\
                         \x20       let ip = i_nl[{s}];\n\
                         \x20       let ig = i_nl[{s1}];\n\
                         \x20       let mut vgk_sum = 0.0f64;\n\
                         \x20       let mut vpk_sum = 0.0f64;\n\
                         \x20       for j in 0..N {{ vgk_sum += N_V[{s}][j] * v[j]; }}\n\
                         \x20       for j in 0..N {{ vpk_sum += N_V[{s1}][j] * v[j]; }}\n\
                         \x20       let p = ip * vpk_sum + ig * vgk_sum;\n"
                    ));
                    code.push_str(&tj_step(dev_num, tp.cth));
                    code.push_str("\x20   }\n");
                }
                _ => {}
            }
        }
    }

    /// Emit device evaluation code WITHOUT declarations (writes to existing i_nl, j_dev).
    ///
    /// `it` is the node iterate the caller extracted `v_nl` from; a MOSFET's
    /// body-effect threshold is taken from the same iterate. When the circuit
    /// has body-effect MOSFETs this also declares `body_gmb` (one entry per
    /// such MOSFET), which the caller must stamp with [`emit_body_gmb_stamp`]
    /// and linearise with [`emit_body_gmb_companion`].
    pub(super) fn emit_nodal_device_evaluation_body(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        it: &str,
    ) {
        let m = ir.topology.m;
        let body = body_effect_mosfets(ir);
        if !body.is_empty() {
            code.push_str(&format!(
                "{indent}let mut body_gmb = [0.0f64; {}];\n",
                body.len()
            ));
        }

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            // Pre-compute flat j_dev index for diagonal (avoids `0 * M + 0` identity_op in generated code)
            let jd_ss = s * m + s;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        // Series resistance: use helper functions
                        let bv_i = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_current(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        let bv_g = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_conductance(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num} (RS={has_rs}, BV={has_bv})\n\
                             {indent}    let (i_rs, g_rs) = diode_eval_with_rs(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS);\n\
                             {indent}    i_nl[{s}] = i_rs{bv_i};\n\
                             {indent}    j_dev[{jd_ss}] = g_rs{bv_g};\n\
                             {indent}}}\n",
                            has_rs = dp.has_rs(), has_bv = dp.has_bv(),
                        ));
                    } else if dp.has_bv() {
                        // Breakdown only (no RS): shared extended-exp helpers +
                        // breakdown. The old inline 40·n_vt clamp made
                        // extreme-IS (wide-bandgap) diodes electrically absent
                        // and used an inconsistent value/derivative pair.
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num} (BV)\n\
                             {indent}    let v = v_nl[{s}];\n\
                             {indent}    i_nl[{s}] = diode_current(v, state.device_{dev_num}_is, state.device_{dev_num}_n_vt) + diode_breakdown_current(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
                             {indent}    j_dev[{jd_ss}] = diode_conductance(v, state.device_{dev_num}_is, state.device_{dev_num}_n_vt) + diode_breakdown_conductance(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
                             {indent}}}\n"
                        ));
                    } else {
                        // Standard diode (no RS, no BV): shared extended-exp helpers
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num}\n\
                             {indent}    i_nl[{s}] = diode_current(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt);\n\
                             {indent}    j_dev[{jd_ss}] = diode_conductance(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt);\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num} (RB/RC/RE inner NR)\n\
                             {indent}    let vbe = v_nl[{s}];\n\
                             {indent}    let vbc = v_nl[{s1}];\n\
                             {indent}    let (ic, ib, jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}    i_nl[{s}] = ic;\n\
                             {indent}    i_nl[{s1}] = ib;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        let mna_note = if slot.has_internal_mna_nodes {
                            " (MNA internal nodes)"
                        } else {
                            ""
                        };
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num}{mna_note}\n\
                             {indent}    let vbe = v_nl[{s}];\n\
                             {indent}    let vbc = v_nl[{s1}];\n\
                             {indent}    let (ic, ib, jac) = bjt_evaluate(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC);\n\
                             {indent}    i_nl[{s}] = ic;\n\
                             {indent}    i_nl[{s1}] = ib;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                    // 1D forward-active BJT: only Vbe→Ic, single jdev entry
                    code.push_str(&format!(
                        "{indent}{{ // BJT {dev_num} forward-active (1D)\n\
                         {indent}    let vbe = v_nl[{s}] * DEVICE_{dev_num}_SIGN;\n\
                         {indent}    let (exp_be, dexp_be) = junction_exp(vbe / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt), state.device_{dev_num}_is);\n\
                         {indent}    i_nl[{s}] = state.device_{dev_num}_is * (exp_be - 1.0) * DEVICE_{dev_num}_SIGN;\n\
                         {indent}    j_dev[{jd_ss}] = state.device_{dev_num}_is / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt) * dexp_be;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                    let s1 = s + 1;
                    let call = super::super::nr_helpers::jfet_evaluate_call(
                        jp, dev_num, "vgs", "vds", "sign",
                    );
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    // jac = [dId/dVgs, dId/dVds, dIg/dVgs, dIg/dVds] but the
                    // NR dims are (s = Vds, s+1 = Vgs), so the columns swap:
                    //   j_dev[s][s]   = dId/dVds = jac[1]
                    //   j_dev[s][s1]  = dId/dVgs = jac[0]
                    //   j_dev[s1][s]  = dIg/dVds = jac[3]
                    //   j_dev[s1][s1] = dIg/dVgs = jac[2]
                    // (mirrors emit_dk_device_eval_for_nodal_schur_indented)
                    code.push_str(&format!(
                        "{indent}{{ // JFET {dev_num}\n\
                         {indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    let (i_d, i_g, jac) = {call};\n\
                         {indent}    i_nl[{s}] = i_d;\n\
                         {indent}    i_nl[{s1}] = i_g;\n\
                         {indent}    j_dev[{jd_ss}] = jac[1];\n\
                         {indent}    j_dev[{jd_01}] = jac[0];\n\
                         {indent}    j_dev[{jd_10}] = jac[3];\n\
                         {indent}    j_dev[{jd_11}] = jac[2];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    // For body effect, compute VT_eff from node voltages at each NR iteration
                    let body_k = body.iter().find(|b| b.1 == dev_num).map(|b| b.0);
                    let vt_expr = if mp.has_body_effect() {
                        let vs_expr = if mp.source_node > 0 {
                            format!("{it}[{}]", mp.source_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let vb_expr = if mp.bulk_node > 0 {
                            format!("{it}[{}]", mp.bulk_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let sign_val = if mp.is_p_channel { -1.0 } else { 1.0 };
                        // Magnitude-space body effect: the GAMMA correction
                        // carries the channel sign so reverse body bias always
                        // increases |VT| (VT stays signed; PMOS VT < 0).
                        code.push_str(&format!(
                            "{indent}{{ // MOSFET {dev_num} body effect\n\
                             {indent}    let vsb = ({sign_val:.1}) * ({vs_expr} - {vb_expr});\n\
                             {indent}    let vt_eff = DEVICE_{dev_num}_VT + ({sign_val:.1}) * DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                             {indent}    state.device_{dev_num}_vt = vt_eff;\n"
                        ));
                        "vt_eff".to_string()
                    } else {
                        code.push_str(&format!("{indent}{{ // MOSFET {dev_num}\n"));
                        format!("state.device_{dev_num}_vt")
                    };
                    let jac_fn = format!(
                            "mosfet_jacobian(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign)"
                    );
                    // jac = [dId/dVgs, dId/dVds, dIg/dVgs, dIg/dVds]; NR dims
                    // are (s = Vds, s+1 = Vgs) — column swap, same as JFET
                    // above and emit_dk_device_eval_for_nodal_schur_indented.
                    // Body effect: dId/dVT = -dId/dVgs, so gmb = gm·dVT/dVsb.
                    let gmb_line = match body_k {
                        Some(k) => format!(
                            "{indent}    body_gmb[{k}] = if vsb > 0.0 {{ jac[0] * DEVICE_{dev_num}_GAMMA / (2.0 * (DEVICE_{dev_num}_PHI + vsb).sqrt()) }} else {{ 0.0 }};\n"
                        ),
                        None => String::new(),
                    };
                    code.push_str(&format!(
                        "{indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    i_nl[{s}] = mosfet_id(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign);\n\
                         {indent}    i_nl[{s1}] = 0.0; // Insulated gate\n\
                         {indent}    let jac = {jac_fn};\n\
                         {indent}    j_dev[{jd_ss}] = jac[1];\n\
                         {indent}    j_dev[{jd_01}] = jac[0];\n\
                         {indent}    j_dev[{jd_10}] = jac[3];\n\
                         {indent}    j_dev[{jd_11}] = jac[2];\n\
                         {gmb_line}{indent}}}\n"
                    ));
                }
                (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    if tp.is_pentode() {
                        // Pentode / beam tetrode NR block. See
                        // [`pentode_dispatch`] for the 8-way helper family
                        // selection (5 sharp/var-mu × 3 grid-off wrappers).
                        let dispatch = pentode_dispatch(tp, dev_num);
                        let helper_suffix = dispatch.suffix;
                        let eval_args = &dispatch.eval_args;
                        if dispatch.is_grid_off {
                            // Grid-off 2D reduction: Ig1 dropped, Vg2k frozen.
                            // Wrapper returns (ip, ig2, [f64;4]) — only 2×2
                            // stamps go into j_dev.
                            code.push_str(&format!(
                                "{indent}{{ // Pentode {dev_num} (grid-off)\n\
                                 {indent}    let vgk = v_nl[{s}];\n\
                                 {indent}    let vpk = v_nl[{s1}];\n\
                                 {indent}    let (ip_t, ig2_t, jac) = tube_evaluate_{helper_suffix}(vgk, vpk, DEVICE_{dev_num}_VG2K_FROZEN, {eval_args});\n\
                                 {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig2_t;\n\
                                 {indent}    j_dev[{jd_ss}] = jac[0];\n\
                                 {indent}    j_dev[{jd_01}] = jac[1];\n\
                                 {indent}    j_dev[{jd_10}] = jac[2];\n\
                                 {indent}    j_dev[{jd_11}] = jac[3];\n\
                                 {indent}}}\n"
                            ));
                        } else {
                            let s2 = s + 2;
                            let jd_02 = s * m + s2;
                            let jd_12 = s1 * m + s2;
                            let jd_20 = s2 * m + s;
                            let jd_21 = s2 * m + s1;
                            let jd_22 = s2 * m + s2;
                            code.push_str(&format!(
                                "{indent}{{ // Pentode {dev_num}\n\
                                 {indent}    let vgk = v_nl[{s}];\n\
                                 {indent}    let vpk = v_nl[{s1}];\n\
                                 {indent}    let vg2k = v_nl[{s2}];\n\
                                 {indent}    let (ip_t, ig2_t, ig1_t, jac) = tube_evaluate_{helper_suffix}(vgk, vpk, vg2k, {eval_args});\n\
                                 {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig2_t; i_nl[{s2}] = ig1_t;\n\
                                 {indent}    j_dev[{jd_ss}] = jac[0];\n\
                                 {indent}    j_dev[{jd_01}] = jac[1];\n\
                                 {indent}    j_dev[{jd_02}] = jac[2];\n\
                                 {indent}    j_dev[{jd_10}] = jac[3];\n\
                                 {indent}    j_dev[{jd_11}] = jac[4];\n\
                                 {indent}    j_dev[{jd_12}] = jac[5];\n\
                                 {indent}    j_dev[{jd_20}] = jac[6];\n\
                                 {indent}    j_dev[{jd_21}] = jac[7];\n\
                                 {indent}    j_dev[{jd_22}] = jac[8];\n\
                                 {indent}}}\n"
                            ));
                        }
                    } else if tp.has_rgi() {
                        // Self-heating Vgk drift: see `nr_helpers.rs`.
                        let vgk_init = if tp.has_self_heating() {
                            format!(
                                "v_nl[{s}] + DEVICE_{dev_num}_VBIAS_ALPHA * (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB)"
                            )
                        } else {
                            format!("v_nl[{s}]")
                        };
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num} (RGI)\n\
                             {indent}    let vgk = {vgk_init};\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    let (ip_t, ig_t, jac) = tube_evaluate_with_rgi(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_gg, state.device_{dev_num}_xi, state.device_{dev_num}_cg, state.device_{dev_num}_lambda, DEVICE_{dev_num}_RGI);\n\
                             {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig_t;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        let vgk_init = if tp.has_self_heating() {
                            format!(
                                "v_nl[{s}] + DEVICE_{dev_num}_VBIAS_ALPHA * (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB)"
                            )
                        } else {
                            format!("v_nl[{s}]")
                        };
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num}\n\
                             {indent}    let vgk = {vgk_init};\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    let (ip_t, ig_t, jac) = tube_evaluate(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_gg, state.device_{dev_num}_xi, state.device_{dev_num}_cg, state.device_{dev_num}_lambda);\n\
                             {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig_t;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    code.push_str(&format!(
                        "{indent}{{ // VCA {dev_num}\n\
                         {indent}    let v_sig = v_nl[{s}];\n\
                         {indent}    let v_ctrl = v_nl[{s1}];\n\
                         {indent}    i_nl[{s}] = vca_current(v_sig, v_ctrl, state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}    i_nl[{s1}] = 0.0;\n\
                         {indent}    let jac = vca_jacobian(v_sig, v_ctrl, state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}    j_dev[{jd_ss}] = jac[0];\n\
                         {indent}    j_dev[{jd_01}] = jac[1];\n\
                         {indent}    j_dev[{jd_10}] = jac[2];\n\
                         {indent}    j_dev[{jd_11}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Ldr, DeviceParams::Ldr(_)) => {
                    // Opto/LDR: linear resistance path, frozen state block.
                    // i_nl = v_nl / r_state, j_dev diagonal = 1/r_state.
                    code.push_str(&format!(
                        "{indent}{{ // LDR {dev_num}\n\
                         {indent}    let ldr_r = state.device_{dev_num}_state[0].max(1e-12);\n\
                         {indent}    i_nl[{s}] = v_nl[{s}] / ldr_r;\n\
                         {indent}    j_dev[{jd_ss}] = 1.0 / ldr_r;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Glow, DeviceParams::Glow(_)) => {
                    // Glow / neon lamp: FROZEN latch selects RS (lit) or ROFF
                    // (dark). Lit is the maintaining line i=(v−V0)/RS
                    // (discharges toward the intercept V0, not ground); dark is a
                    // resistor through the origin. j_dev diagonal = 1/R (the V0
                    // term is affine).
                    code.push_str(&format!(
                        "{indent}{{ // GLOW {dev_num}\n\
                         {indent}    let glow_lit = state.device_{dev_num}_state[0] >= 0.5;\n\
                         {indent}    let glow_r = if glow_lit {{ DEVICE_{dev_num}_RS }} else {{ DEVICE_{dev_num}_ROFF }};\n\
                         {indent}    let glow_emf = if glow_lit {{ DEVICE_{dev_num}_V0 }} else {{ 0.0 }};\n\
                         {indent}    i_nl[{s}] = (v_nl[{s}] - glow_emf) / glow_r;\n\
                         {indent}    j_dev[{jd_ss}] = 1.0 / glow_r;\n\
                         {indent}}}\n"
                    ));
                }
                _ => {} // Mismatched type/params — skip
            }
        }
    }

    /// Emit final device evaluation at converged point (writes into existing `i_nl`).
    /// `it` is the node iterate `v_nl_final` was extracted from; a MOSFET's
    /// body-effect threshold is recomputed from it rather than read from the
    /// last Newton iteration's value.
    pub(super) fn emit_nodal_device_evaluation_final(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        it: &str,
    ) {
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        let bv_i = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_current(v_nl_final[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = diode_current_with_rs(v_nl_final[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_i};\n"
                        ));
                    } else if dp.has_bv() {
                        // Shared extended-exp helper (matches NR-loop stamp).
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = diode_current(v_nl_final[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt) + diode_breakdown_current(v_nl_final[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = diode_current(v_nl_final[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt);\n"
                        ));
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                    let s1 = s + 1;
                    if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}];\n\
                             {indent}  let vbc = v_nl_final[{s1}];\n\
                             {indent}  let (ic, ib, _jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}  i_nl[{s}] = ic;\n\
                             {indent}  i_nl[{s1}] = ib;\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}];\n\
                             {indent}  let vbc = v_nl_final[{s1}];\n\
                             {indent}  let (ic, ib, _) = bjt_evaluate(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC);\n\
                             {indent}  i_nl[{s}] = ic;\n\
                             {indent}  i_nl[{s1}] = ib;\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                    // 1D forward-active BJT: only Vbe→Ic
                    code.push_str(&format!(
                        "{indent}{{ let vbe = v_nl_final[{s}] * DEVICE_{dev_num}_SIGN;\n\
                         {indent}  let exp_be = junction_exp(vbe / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt), state.device_{dev_num}_is).0;\n\
                         {indent}  i_nl[{s}] = state.device_{dev_num}_is * (exp_be - 1.0) * DEVICE_{dev_num}_SIGN;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                    let s1 = s + 1;
                    let call = super::super::nr_helpers::jfet_evaluate_call(
                        jp,
                        dev_num,
                        &format!("v_nl_final[{s1}]"),
                        &format!("v_nl_final[{s}]"),
                        &format!("DEVICE_{dev_num}_SIGN"),
                    );
                    code.push_str(&format!(
                        "{indent}{{ let (i_d, i_g, _) = {call}; i_nl[{s}] = i_d; i_nl[{s1}] = i_g; }}\n"
                    ));
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                    let s1 = s + 1;
                    let vt = if mp.has_body_effect() {
                        let node = |idx: usize| {
                            if idx > 0 {
                                format!("{it}[{}]", idx - 1)
                            } else {
                                "0.0".to_string()
                            }
                        };
                        let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                        format!(
                            "{{ let vsb = ({sign:.1}) * ({} - {}); DEVICE_{dev_num}_VT + ({sign:.1}) * DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt()) }}",
                            node(mp.source_node),
                            node(mp.bulk_node)
                        )
                    } else {
                        format!("state.device_{dev_num}_vt")
                    };
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = mosfet_id(v_nl_final[{s1}], v_nl_final[{s}], state.device_{dev_num}_kp, {vt}, state.device_{dev_num}_lambda, DEVICE_{dev_num}_SIGN);\n\
                         {indent}i_nl[{s1}] = 0.0;\n"
                    ));
                }
                (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                    let s1 = s + 1;
                    if tp.is_pentode() {
                        let dispatch = pentode_dispatch(tp, dev_num);
                        let helper_suffix = dispatch.suffix;
                        let ip_args = &dispatch.ip_args;
                        let is_args = &dispatch.is_args;
                        if dispatch.is_grid_off {
                            // Grid-off 2D: read Vg2k from the frozen constant,
                            // stamp only Ip and Ig2. Ig1 is identically zero
                            // and the slot contributes no s+2 dimension.
                            code.push_str(&format!(
                                "{indent}i_nl[{s}] = tube_ip_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], DEVICE_{dev_num}_VG2K_FROZEN, {ip_args});\n\
                                 {indent}i_nl[{s1}] = tube_is_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], DEVICE_{dev_num}_VG2K_FROZEN, {is_args});\n"
                            ));
                        } else {
                            let s2 = s + 2;
                            code.push_str(&format!(
                                "{indent}i_nl[{s}] = tube_ip_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], v_nl_final[{s2}], {ip_args});\n\
                                 {indent}i_nl[{s1}] = tube_is_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], v_nl_final[{s2}], {is_args});\n\
                                 {indent}i_nl[{s2}] = tube_ig(v_nl_final[{s}], state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset);\n"
                            ));
                        }
                    } else if tp.has_rgi() {
                        // Final eval matches NR stamp: Vgk is thermally shifted
                        // when `has_self_heating()`. The grid-current path
                        // `tube_ig_with_rgi` also sees the shifted Vgk for
                        // contact-potential consistency.
                        let vgk_fe = if tp.has_self_heating() {
                            format!(
                                "(v_nl_final[{s}] + DEVICE_{dev_num}_VBIAS_ALPHA * (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB))"
                            )
                        } else {
                            format!("v_nl_final[{s}]")
                        };
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = tube_ip_with_rgi({vgk_fe}, v_nl_final[{s1}], state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda, state.device_{dev_num}_gg, state.device_{dev_num}_xi, state.device_{dev_num}_cg, DEVICE_{dev_num}_RGI);\n\
                             {indent}i_nl[{s1}] = tube_ig_dz_with_rgi({vgk_fe}, state.device_{dev_num}_gg, state.device_{dev_num}_xi, state.device_{dev_num}_cg, DEVICE_{dev_num}_RGI);\n"
                        ));
                    } else {
                        let vgk_fe = if tp.has_self_heating() {
                            format!(
                                "(v_nl_final[{s}] + DEVICE_{dev_num}_VBIAS_ALPHA * (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB))"
                            )
                        } else {
                            format!("v_nl_final[{s}]")
                        };
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = tube_ip({vgk_fe}, v_nl_final[{s1}], state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda);\n\
                             {indent}i_nl[{s1}] = tube_ig_dz({vgk_fe}, state.device_{dev_num}_gg, state.device_{dev_num}_xi, state.device_{dev_num}_cg);\n"
                        ));
                    }
                }
                (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = vca_current(v_nl_final[{s}], v_nl_final[{s1}], state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}i_nl[{s1}] = 0.0;\n"
                    ));
                }
                (DeviceType::Ldr, DeviceParams::Ldr(_)) => {
                    // Opto/LDR: i_nl = v / r_state at the converged voltage.
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = v_nl_final[{s}] / state.device_{dev_num}_state[0].max(1e-12);\n"
                    ));
                }
                (DeviceType::Glow, DeviceParams::Glow(_)) => {
                    // Glow: i_nl at the converged voltage. Lit is the
                    // maintaining line i=(v−V0)/RS; dark is v/ROFF.
                    code.push_str(&format!(
                        "{indent}{{ let glow_lit = state.device_{dev_num}_state[0] >= 0.5;\n\
                         {indent}  let glow_r = if glow_lit {{ DEVICE_{dev_num}_RS }} else {{ DEVICE_{dev_num}_ROFF }};\n\
                         {indent}  let glow_emf = if glow_lit {{ DEVICE_{dev_num}_V0 }} else {{ 0.0 }};\n\
                         {indent}  i_nl[{s}] = (v_nl_final[{s}] - glow_emf) / glow_r; }}\n"
                    ));
                }
                _ => {}
            }
        }
    }

    /// Emit SPICE voltage limiting for nodal solver (trapezoidal NR, at default indent).
    pub(super) fn emit_nodal_voltage_limiting(code: &mut String, ir: &CircuitIR) {
        Self::emit_nodal_voltage_limiting_indented(code, ir, "        ");
    }

    /// Emit SPICE voltage limiting for nodal solver at a given indent level.
    ///
    /// For each nonlinear device dimension, computes the proposed device voltage
    /// from v_new via N_v, applies pnjlim/fetlim, and reduces alpha if needed.
    pub(super) fn emit_nodal_voltage_limiting_indented(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
    ) {
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            for d in 0..slot.dimension {
                let i = slot.start_idx + d;

                // Compute proposed device voltage from v_new via N_v
                code.push_str(&format!("{indent}{{ // Device {dev_num} dim {d}\n"));
                code.push_str(&emit_sparse_nv_dot(
                    ir,
                    i,
                    "v_nl_proposed",
                    "v_new",
                    &format!("{indent}    "),
                ));
                code.push_str(&format!("{indent}    let v_nl_current = v_nl[{i}];\n"));
                code.push_str(&format!(
                    "{indent}    let dv = v_nl_proposed - v_nl_current;\n"
                ));
                // Same 1e-4 threshold as the transient path — pnjlim/fetlim are
                // no-ops for |dv| < 2·Vt ≈ 52 mV, so 0.1 mV is safely conservative.
                code.push_str(&format!("{indent}    if dv.abs() > 1e-4 {{\n"));

                // Apply per-device limiter
                match (&slot.device_type, d) {
                    (DeviceType::Diode, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Bjt, _) | (DeviceType::BjtForwardActive, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Jfet, _) => {
                        // Both gate junctions need the other dimension's voltage.
                        let (s, s1) = (slot.start_idx, slot.start_idx + 1);
                        let other = if d == 0 { s1 } else { s };
                        code.push_str(&emit_sparse_nv_dot(
                            ir,
                            other,
                            "v_nl_proposed_other",
                            "v_new",
                            &format!("{indent}        "),
                        ));
                        let (new_ds, old_ds, new_gs, old_gs) = if d == 0 {
                            (
                                "v_nl_proposed".to_string(),
                                "v_nl_current".to_string(),
                                "v_nl_proposed_other".to_string(),
                                format!("v_nl[{s1}]"),
                            )
                        } else {
                            (
                                "v_nl_proposed_other".to_string(),
                                format!("v_nl[{s}]"),
                                "v_nl_proposed".to_string(),
                                "v_nl_current".to_string(),
                            )
                        };
                        let lim = super::super::nr_helpers::jfet_limit_expr(
                            dev_num, d, &new_ds, &old_ds, &new_gs, &old_gs,
                        );
                        code.push_str(&format!("{indent}        let v_lim = {lim};\n"));
                    }
                    (DeviceType::Mosfet, 0) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
                    }
                    (DeviceType::Mosfet, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vt);\n"
                        ));
                    }
                    (DeviceType::Tube, 0) => {
                        let grid_vt =
                            super::super::helpers::tube_grid_vt_expr(&slot.params, dev_num);
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, {grid_vt}, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Tube, 2) => {
                        let grid_vt =
                            super::super::helpers::tube_grid_vt_expr(&slot.params, dev_num);
                        // Pentode dim 2 = Vg2k — log-junction limiting (see DK NR limiter).
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, {grid_vt}, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Tube, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
                    }
                    (DeviceType::Vca, _) => {
                        // VCA: no junction limiting needed — fast_exp already clamps
                        code.push_str(&format!("{indent}        let v_lim = v_nl_proposed;\n"));
                    }
                    (DeviceType::Ldr, _) => {
                        // LDR: linear resistance path — no limiting.
                        code.push_str(&format!("{indent}        let v_lim = v_nl_proposed;\n"));
                    }
                    (DeviceType::Glow, _) => {
                        // Glow: monotone resistance path — no limiting.
                        code.push_str(&format!("{indent}        let v_lim = v_nl_proposed;\n"));
                    }
                }

                code.push_str(&format!(
                    "{indent}        let ratio = ((v_lim - v_nl_current) / dv).clamp(0.01, 1.0);\n\
                     {indent}        if ratio < alpha {{ alpha = ratio; }}\n\
                     {indent}    }}\n\
                     {indent}}}\n"
                ));
            }
        }
    }
}
