//! Device-model function emission shared by the DK and nodal paths.

use tera::Context;

use super::helpers::{
    emit_device_const, emit_stateful_update_fns, fmt_f64, section_banner, stateful_device_data,
};
use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceParams};
use crate::codegen::CodegenError;

impl RustEmitter {
    pub(super) fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;
        let mut has_jfet = false;
        let mut has_jfet_ps = false;
        let mut has_mosfet = false;
        let mut has_tube = false;
        let mut has_vca = false;
        let mut has_self_heating = false;
        // True when ≥1 glow device carries active relaxing sections — gates the
        // `device_glow` helper (inner-Newton lit eval) and the new consts. False
        // for every no-section glow deck, keeping their emitted code byte-identical.
        let mut has_glow_sections = false;
        // True when ≥1 glow device carries ignition depression (D_AMP≠0) — gates
        // the `glow_D` helper and D consts. Independent of sections.
        let mut has_glow_d = false;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                DeviceParams::Diode(dp) => {
                    has_diode = true;
                    if dp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "IS", dp.is);
                    emit_device_const(&mut code, dev_num, "N_VT", dp.n_vt);
                    // Precomputed critical voltage for SPICE pnjlim
                    let vcrit = dp.n_vt * (dp.n_vt / (std::f64::consts::SQRT_2 * dp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    if dp.has_rs() {
                        emit_device_const(&mut code, dev_num, "RS", dp.rs);
                    }
                    if dp.has_bv() {
                        emit_device_const(&mut code, dev_num, "BV", dp.bv);
                        emit_device_const(&mut code, dev_num, "IBV", dp.ibv);
                    }
                    if dp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", dp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", dp.cth);
                        emit_device_const(&mut code, dev_num, "XTI", dp.xti);
                        emit_device_const(&mut code, dev_num, "EG", dp.eg);
                        emit_device_const(&mut code, dev_num, "TAMB", dp.tamb);
                        emit_device_const(&mut code, dev_num, "IS_NOM", dp.is);
                        emit_device_const(&mut code, dev_num, "N_VT_NOM", dp.n_vt);
                    }
                    code.push('\n');
                }
                DeviceParams::Bjt(bp) => {
                    has_bjt = true;
                    if bp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "IS", bp.is);
                    emit_device_const(&mut code, dev_num, "VT", bp.vt);
                    emit_device_const(&mut code, dev_num, "BETA_F", bp.beta_f);
                    emit_device_const(&mut code, dev_num, "BETA_R", bp.beta_r);
                    emit_device_const(&mut code, dev_num, "NF", bp.nf);
                    emit_device_const(&mut code, dev_num, "NR", bp.nr);
                    emit_device_const(&mut code, dev_num, "ISE", bp.ise);
                    emit_device_const(&mut code, dev_num, "NE", bp.ne);
                    emit_device_const(&mut code, dev_num, "ISC", bp.isc);
                    emit_device_const(&mut code, dev_num, "NC", bp.nc);
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n",
                        dev_num, sign
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_USE_GP: bool = {};\n",
                        dev_num,
                        bp.is_gummel_poon()
                    ));
                    emit_device_const(&mut code, dev_num, "VAF", bp.vaf);
                    emit_device_const(&mut code, dev_num, "VAR", bp.var);
                    emit_device_const(&mut code, dev_num, "IKF", bp.ikf);
                    emit_device_const(&mut code, dev_num, "IKR", bp.ikr);
                    // Precomputed critical voltage for SPICE pnjlim (both Vbe and Vbc junctions)
                    let vcrit = bp.vt * (bp.vt / (std::f64::consts::SQRT_2 * bp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    // Read by K_eff (DK) or bjt_with_parasitics (the DC-OP
                    // recompute, nodal without internal nodes). A device with
                    // internal nodes carries RB/RC/RE as conductances in G.
                    if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                        emit_device_const(&mut code, dev_num, "RB", bp.rb);
                        emit_device_const(&mut code, dev_num, "RC", bp.rc);
                        emit_device_const(&mut code, dev_num, "RE", bp.re);
                    }
                    if bp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", bp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", bp.cth);
                        emit_device_const(&mut code, dev_num, "XTI", bp.xti);
                        emit_device_const(&mut code, dev_num, "XTB", bp.xtb);
                        emit_device_const(&mut code, dev_num, "EG", bp.eg);
                        emit_device_const(&mut code, dev_num, "TAMB", bp.tamb);
                        emit_device_const(&mut code, dev_num, "IS_NOM", bp.is);
                    }
                    code.push('\n');
                }
                DeviceParams::Jfet(jp) => {
                    has_jfet = true;
                    if let Some(ps) = &jp.ps {
                        // LEVEL=2: BETA replaces IDSS; the shape keys travel
                        // as one array, the order `jfet_ps_evaluate` reads.
                        has_jfet_ps = true;
                        emit_device_const(&mut code, dev_num, "BETA", ps.beta);
                        code.push_str(&format!(
                            "const DEVICE_{dev_num}_PS: [f64; 8] = [{}]; // VST MVST P Q Z XI MXI PB\n",
                            [ps.vst, ps.mvst, ps.p, ps.q, ps.z, ps.xi, ps.mxi, ps.vbi]
                                .map(fmt_f64)
                                .join(", ")
                        ));
                    } else {
                        emit_device_const(&mut code, dev_num, "IDSS", jp.idss);
                    }
                    emit_device_const(&mut code, dev_num, "VP", jp.vp);
                    emit_device_const(&mut code, dev_num, "LAMBDA", jp.lambda);
                    // Gate junctions: IS, N*Vt and the pnjlim critical voltage.
                    emit_device_const(&mut code, dev_num, "IS", jp.is);
                    emit_device_const(&mut code, dev_num, "N_VT", jp.gate_n_vt());
                    let vcrit = if jp.is > 0.0 {
                        melange_primitives::nr::pn_vcrit(jp.gate_n_vt(), jp.is)
                    } else {
                        f64::MAX
                    };
                    emit_device_const(&mut code, dev_num, "GATE_VCRIT", vcrit);
                    let sign = if jp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Mosfet(mp) => {
                    has_mosfet = true;
                    emit_device_const(&mut code, dev_num, "KP", mp.kp);
                    emit_device_const(&mut code, dev_num, "VT", mp.vt);
                    emit_device_const(&mut code, dev_num, "LAMBDA", mp.lambda);
                    if mp.has_body_effect() {
                        emit_device_const(&mut code, dev_num, "GAMMA", mp.gamma);
                        emit_device_const(&mut code, dev_num, "PHI", mp.phi);
                    }
                    let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Tube(tp) => {
                    has_tube = true;
                    if tp.has_self_heating() {
                        has_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "MU", tp.mu);
                    emit_device_const(&mut code, dev_num, "EX", tp.ex);
                    emit_device_const(&mut code, dev_num, "KG1", tp.kg1);
                    emit_device_const(&mut code, dev_num, "KP", tp.kp);
                    emit_device_const(&mut code, dev_num, "KVB", tp.kvb);
                    // Grid-current law. The two tube families no longer share
                    // one: triodes carry Dempwolf & Zölzer eq. (11) (Gg/xi/Cg),
                    // pentode control grids still carry Leach (IG_MAX/
                    // VGK_ONSET). Emitting only the pair the device actually
                    // uses keeps a wrong constant from sitting in the generated
                    // file looking authoritative.
                    if tp.is_pentode() {
                        emit_device_const(&mut code, dev_num, "IG_MAX", tp.ig_max);
                        emit_device_const(&mut code, dev_num, "VGK_ONSET", tp.vgk_onset);
                    } else {
                        emit_device_const(&mut code, dev_num, "GG", tp.gg);
                        emit_device_const(&mut code, dev_num, "XI", tp.xi);
                        emit_device_const(&mut code, dev_num, "CG", tp.cg);
                    }
                    // The triode's Early term; the pentode plate law has none.
                    if !tp.is_pentode() {
                        emit_device_const(&mut code, dev_num, "LAMBDA", tp.lambda);
                    }
                    if tp.has_rgi() {
                        emit_device_const(&mut code, dev_num, "RGI", tp.rgi);
                    }
                    // Pentode-only constants (Reefman Derk §4.4). Triodes leave
                    // these unset so the emitted output is byte-identical for
                    // pure-triode circuits.
                    if tp.is_pentode() {
                        emit_device_const(&mut code, dev_num, "KG2", tp.kg2);
                        emit_device_const(&mut code, dev_num, "ALPHA_S", tp.alpha_s);
                        emit_device_const(&mut code, dev_num, "A_FACTOR", tp.a_factor);
                        emit_device_const(&mut code, dev_num, "BETA_FACTOR", tp.beta_factor);
                    }
                    // Grid-off reduced pentode constant: the DC-OP-converged
                    // screen voltage Vg2k that the 2D reduced NR block uses in
                    // place of the dropped NR dimension. Read from the
                    // `DeviceSlot`, not `TubeParams` — it's runtime state
                    // captured by the DC-OP grid-off detection pass.
                    if tp.is_grid_off_pentode() {
                        emit_device_const(&mut code, dev_num, "VG2K_FROZEN", slot.vg2k_frozen);
                    }
                    // Variable-mu §5 constants. Emitted only when `svar > 0`
                    // to preserve byte-identity for sharp (phase 1a/1a.1)
                    // circuits. Applies to BOTH triodes and pentodes.
                    if tp.is_variable_mu() {
                        emit_device_const(&mut code, dev_num, "MU_B", tp.mu_b);
                        emit_device_const(&mut code, dev_num, "SVAR", tp.svar);
                        emit_device_const(&mut code, dev_num, "EX_B", tp.ex_b);
                    }
                    // Precomputed critical voltage for SPICE pnjlim on the grid
                    // dimension. The scale is the grid law's own turn-on width:
                    // `1/Cg` for the D&Z triode (the softplus adaption factor is
                    // a reciprocal voltage, exactly the role `n·Vt` plays for a
                    // diode), `vgk_onset/3` for the Leach pentode control grid.
                    // Must agree with `helpers::tube_grid_vt_expr`, which emits
                    // the matching runtime expression.
                    let vt_tube = if tp.is_pentode() {
                        tp.vgk_onset / 3.0
                    } else {
                        1.0 / tp.cg
                    };
                    let vcrit = vt_tube * (vt_tube / (std::f64::consts::SQRT_2 * 1e-10)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    // Self-heating constants. Only the thermal gate (RTH) is
                    // checked here — `has_self_heating()` is already false for
                    // pentodes, so this block is triode-only in phase 1.
                    if tp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", tp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", tp.cth);
                        emit_device_const(&mut code, dev_num, "VBIAS_ALPHA", tp.vbias_alpha);
                        emit_device_const(&mut code, dev_num, "TAMB", tp.tamb);
                    }
                    code.push('\n');
                }
                DeviceParams::Vca(vp) => {
                    has_vca = true;
                    emit_device_const(&mut code, dev_num, "VSCALE", vp.vscale);
                    emit_device_const(&mut code, dev_num, "G0", vp.g0);
                    emit_device_const(&mut code, dev_num, "THD", vp.thd);
                    code.push('\n');
                }
                DeviceParams::Ldr(lp) => {
                    // Opto/LDR: model params consumed by the after-solve update()
                    // hook (target-R power law + attack/release taus). The live
                    // resistance is the opaque `device_{n}_state` block, not a
                    // const. The NR eval reads `1/state[0]` — no const needed there.
                    emit_device_const(&mut code, dev_num, "RMIN", lp.r_min);
                    emit_device_const(&mut code, dev_num, "RMAX", lp.r_max);
                    emit_device_const(&mut code, dev_num, "GAMMA", lp.gamma);
                    emit_device_const(&mut code, dev_num, "TAU_A", lp.attack_tau);
                    emit_device_const(&mut code, dev_num, "TAU_R", lp.release_tau);
                    code.push('\n');
                }
                DeviceParams::Glow(gp) => {
                    // Glow / neon lamp (EXPERIMENTAL). VO = strike threshold
                    // (dark→lit, after-solve update()). The lit maintaining line
                    // is V0 + RS·i (a voltage source with a soft positive slope,
                    // NOT a resistor to ground): the in-NR eval is a Thévenin
                    // source i=(v−V0)/RS so the reservoir discharges toward the
                    // INTERCEPT V0 (= datasheet VM − RS·IK, NOT the static VM),
                    // and update() extinguishes on holding current
                    // (v−V0)/RS ≤ IHOLD. RS/ROFF are the lit / dark resistances
                    // (selected by the frozen latch). The live latch is the
                    // opaque `device_{n}_state` block, not a const.
                    emit_device_const(&mut code, dev_num, "VO", gp.vo);
                    emit_device_const(&mut code, dev_num, "V0", gp.v0);
                    emit_device_const(&mut code, dev_num, "RS", gp.rs);
                    emit_device_const(&mut code, dev_num, "ROFF", gp.roff);
                    emit_device_const(&mut code, dev_num, "IHOLD", gp.ihold);
                    // Relaxing-section lit branch (defaults-off). Emitted ONLY
                    // for section-bearing devices → the const block for every
                    // no-section glow deck is byte-identical to before. RT = DC
                    // asymptote; K{i}/TAU{i} = the delayed-overvoltage stack;
                    // IFLOOR = the log-domain current clamp (= IHOLD, the
                    // analog of safe_exp for ln), also the Ī_i seed floor.
                    if gp.has_sections() {
                        has_glow_sections = true;
                        emit_device_const(&mut code, dev_num, "RT", gp.r_t);
                        for i in 0..crate::device_types::GlowParams::MAX_SECTIONS {
                            emit_device_const(&mut code, dev_num, &format!("K{}", i + 1), gp.k[i]);
                            emit_device_const(
                                &mut code,
                                dev_num,
                                &format!("TAU{}", i + 1),
                                gp.tau[i],
                            );
                        }
                        emit_device_const(&mut code, dev_num, "IFLOOR", gp.ifloor);
                        // Subnormal branch (part-a; default 0 = off). I_N anchors
                        // the static −KSUB·ln(I/I_N) term at the rated current.
                        emit_device_const(&mut code, dev_num, "KSUB", gp.ksub);
                        emit_device_const(&mut code, dev_num, "I_N", gp.i_n);
                    }
                    // Ignition depression (Part B; defaults-off). Emitted ONLY
                    // for D-bearing devices → non-D glow decks stay byte-identical.
                    // D_CAP = VO−VM is the hard cap (V_s,eff never below VM).
                    if gp.has_d() {
                        has_glow_d = true;
                        emit_device_const(&mut code, dev_num, "D_AMP", gp.d_amp);
                        emit_device_const(&mut code, dev_num, "D_TKNEE", gp.d_tknee);
                        emit_device_const(&mut code, dev_num, "D_THOLD", gp.d_thold);
                        emit_device_const(&mut code, dev_num, "D_CAP", gp.d_cap);
                    }
                    code.push('\n');
                }
            }
        }

        // Boltzmann constant / elementary charge (k/q in eV/K)
        if has_self_heating {
            code.push_str("/// Boltzmann constant / elementary charge [eV/K]\n");
            code.push_str("const BOLTZMANN_Q: f64 = 8.617333262e-5;\n\n");
        }

        // Fast exp() approximation (needed by all nonlinear device models)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&Self::emit_fast_exp());
        }

        // SPICE voltage limiting functions (needed by all nonlinear devices)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&self.render("spice_limiting", &Context::new())?);
        }

        if has_diode {
            code.push_str(&self.render("device_diode", &Context::new())?);
        }
        // The one junction exponential shared by the BJT and the JFET gate.
        if has_bjt || has_jfet {
            code.push_str(&self.render("junction_exp", &Context::new())?);
        }
        if has_bjt {
            code.push_str(&self.render("device_bjt", &Context::new())?);
        }
        if has_jfet {
            code.push_str(&self.render("device_jfet", &Context::new())?);
            if has_jfet_ps {
                code.push_str(&self.render("device_jfet_ps", &Context::new())?);
            }
        }
        if has_mosfet {
            code.push_str(&self.render("device_mosfet", &Context::new())?);
        }
        if has_tube {
            // Five Tera guards for pentode-family tubes:
            //   any_pentode            — sharp Rational (Derk §4.4): EL84/EL34/EF86
            //   any_beam_tetrode       — sharp Exponential (DerkE §4.5): 6L6GC/6V6GT
            //   any_variable_mu_pentode       — §5 two-section Rational: 6K7
            //   any_variable_mu_beam_tetrode  — §5 two-section Exponential: EF89
            //   any_classical_pentode  — Classical Norman Koren (Cohen-Hélie §2): KT88/6550
            //
            // Byte-identity guarantee: a circuit containing only non-pentode tubes
            // (pure-triode) has all five false and emits the phase-1a triode block
            // unchanged. A circuit containing only sharp pentodes (svar=0) has
            // `any_variable_mu_*` and `any_classical_pentode` false and emits
            // phase-1a.1 output byte-identical. A pure-Derk (Rational/Exponential)
            // circuit has `any_classical_pentode` false and emits phase-1c output
            // byte-identical. Variable-mu Classical is rejected by
            // `TubeParams::validate()`, so no `any_variable_mu_classical_pentode`
            // flag exists.
            // Grid-off pentode guard: excluded from the sharp flags below so
            // the existing sharp / variable-mu / Classical helper families
            // stay byte-identical for circuits with zero grid-off slots.
            // `any_grid_off_pentode` gates a fresh block of thin 2D wrapper
            // helpers that delegate back to the sharp 3D helpers with
            // `vg2k_frozen` substituted for the live Vg2k dimension.
            let any_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_classical_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical)
                        && !tp.is_grid_off_pentode())
            });
            let any_grid_off_pentode = ir.device_slots.iter().any(
                |slot| matches!(&slot.params, DeviceParams::Tube(tp) if tp.is_grid_off_pentode()),
            );
            // Grid-off wrapper helpers delegate to the sharp 3D helpers of
            // the matching screen form, so whichever sharp family a grid-off
            // slot uses MUST have its own helper emitted too. Force the
            // matching sharp flag on whenever a grid-off slot exists of that
            // screen form. This keeps the template simple (a single
            // `{% if any_grid_off_pentode %}` block can reference both the
            // wrapper and the 3D helper it delegates to) without leaking
            // sharp helpers into circuits that have neither.
            let any_grid_off_rational = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational))
            });
            let any_grid_off_exponential = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential))
            });
            let any_grid_off_classical = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical))
            });
            let any_pentode = any_pentode || any_grid_off_rational;
            let any_beam_tetrode = any_beam_tetrode || any_grid_off_exponential;
            let any_classical_pentode = any_classical_pentode || any_grid_off_classical;
            // The Leach control-grid helpers (`tube_ig` / `tube_ig_deriv`) are
            // shared by EVERY pentode family, so they need a flag that is the
            // union of all of them — the per-family flags below each gate only
            // their own equation set. Triode-only circuits emit neither helper:
            // the triode grid law is now `tube_ig_dz` (D&Z eq. 11).
            let any_pentode_family = any_pentode
                || any_beam_tetrode
                || any_variable_mu_pentode
                || any_variable_mu_beam_tetrode
                || any_classical_pentode
                || any_grid_off_pentode;
            let mut tube_ctx = Context::new();
            tube_ctx.insert("any_pentode_family", &any_pentode_family);
            tube_ctx.insert("any_pentode", &any_pentode);
            tube_ctx.insert("any_beam_tetrode", &any_beam_tetrode);
            tube_ctx.insert("any_variable_mu_pentode", &any_variable_mu_pentode);
            tube_ctx.insert(
                "any_variable_mu_beam_tetrode",
                &any_variable_mu_beam_tetrode,
            );
            tube_ctx.insert("any_classical_pentode", &any_classical_pentode);
            tube_ctx.insert("any_grid_off_pentode", &any_grid_off_pentode);
            tube_ctx.insert("any_grid_off_rational", &any_grid_off_rational);
            tube_ctx.insert("any_grid_off_exponential", &any_grid_off_exponential);
            tube_ctx.insert("any_grid_off_classical", &any_grid_off_classical);
            code.push_str(&self.render("device_tube", &tube_ctx)?);
        }
        if has_vca {
            code.push_str(&self.render("device_vca", &Context::new())?);
        }
        // Glow helpers: the inner-Newton lit eval (`glow_lit_eval`, sections)
        // and/or the ignition-depression curve (`glow_D`). Each fn is gated
        // inside the template, and the template is rendered only when at least
        // one is needed, so no-section-no-D glow decks (and every non-glow deck)
        // never see either helper.
        if has_glow_sections || has_glow_d {
            let mut glow_ctx = Context::new();
            glow_ctx.insert("emit_lit_eval", &has_glow_sections);
            glow_ctx.insert("emit_d", &has_glow_d);
            code.push_str(&self.render("device_glow", &glow_ctx)?);
        }

        // Stateful-device (Phase 0c) update() hooks. Shared by BOTH emitters —
        // `emit_device_models` is called from the DK and nodal generate paths —
        // so the update signature and body can never drift. Empty when no
        // device is stateful (byte-identical to before this machinery existed).
        code.push_str(&emit_stateful_update_fns(
            &stateful_device_data(ir),
            &ir.device_slots,
            // Sub-sample fire form (both latch flips report a crossing fraction)
            // only where the nodal-Schur event loop consumes it; `false` on every
            // other deck keeps the reserved-slot form byte-for-byte.
            ir.solver_config.subsample_fire,
        ));

        Ok(code)
    }

    /// Emit fast exp() approximation function.
    ///
    /// Default: polynomial range reduction + 5th-order minimax (~6 cycles, <0.0004% error).
    /// Opt-in: `--cfg melange_precise_exp` uses hardware/libm exp (~38 cycles).
    ///
    /// Accuracy of polynomial path: <0.0004% max relative error over [-40, 40].
    pub(super) fn emit_fast_exp() -> String {
        let mut code = String::new();
        code.push_str(
            "/// Fast exp() for audio circuit simulation.\n\
             /// Input clamped to [-40, 40] (matches melange safe_exp convention).\n\
             ///\n\
             /// Default: polynomial approximation (<0.0004% error, ~6x faster than libm).\n\
             /// To use hardware/libm exp, compile with: `--cfg melange_precise_exp`\n\
             #[inline(always)]\n\
             fn fast_exp(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.clamp(-40.0, 40.0).exp() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Range reduction + 5th-order minimax polynomial. <0.0004% max relative error.\n\
             \x20       // No lookup tables, no libm dependency, branchless hot path.\n\
             \x20       let x = x.clamp(-40.0, 40.0);\n\
             \x20       const LN2_INV: f64 = std::f64::consts::LOG2_E;\n\
             \x20       const LN2_HI: f64 = 0.6931471803691238;\n\
             \x20       const LN2_LO: f64 = 1.9082149292705877e-10;\n\
             \x20       const SHIFT: f64 = 6755399441055744.0; // 2^52 + 2^51\n\
             \x20       let z = x * LN2_INV + SHIFT;\n\
             \x20       let n_i64 = z.to_bits() as i64 - SHIFT.to_bits() as i64;\n\
             \x20       let n = n_i64 as f64;\n\
             \x20       let f = (x - n * LN2_HI) - n * LN2_LO;\n\
             \x20       let p = 1.0 + f * (1.0 + f * (0.5 + f * (0.16666666666666607\n\
             \x20           + f * (0.04166666666665876 + f * 0.008333333333492337))));\n\
             \x20       let pow2n = f64::from_bits(((1023 + n_i64) as u64) << 52);\n\
             \x20       p * pow2n\n\
             \x20   }\n\
             }\n\n",
        );
        // fast_ln: used in tube softplus ln(1+exp(x))
        code.push_str(
            "/// Fast ln() for audio circuit simulation.\n\
             /// Symmetric log series (~0.005% max relative error, ~3x faster than libm).\n\
             /// Only valid for positive inputs.\n\
             #[inline(always)]\n\
             fn fast_ln(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.ln() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Extract exponent and mantissa from IEEE 754 double\n\
             \x20       let bits = x.to_bits();\n\
             \x20       let e = ((bits >> 52) & 0x7FF) as i64 - 1023;\n\
             \x20       // Normalize mantissa to [1, 2)\n\
             \x20       let m_bits = (bits & 0x000F_FFFF_FFFF_FFFF) | 0x3FF0_0000_0000_0000;\n\
             \x20       let m = f64::from_bits(m_bits);\n\
             \x20       // Symmetric log series: u = (m-1)/(m+1), ln(m) = 2u(1 + u²/3 + u⁴/5 + u⁶/7)\n\
             \x20       // For m in [1,2), u in [0,1/3): converges much faster than Taylor.\n\
             \x20       let u = (m - 1.0) / (m + 1.0);\n\
             \x20       let u2 = u * u;\n\
             \x20       let ln_m = 2.0 * u * (1.0 + u2 * (0.3333333333333333 + u2 * (0.2 + u2 * 0.14285714285714285)));\n\
             \x20       ln_m + (e as f64) * std::f64::consts::LN_2\n\
             \x20   }\n\
             }\n\n",
        );
        code
    }
}
