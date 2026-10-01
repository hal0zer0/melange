//! VCA, LDR and glow-lamp `.model` parameter resolution.

use super::*;

impl CircuitIR {
    /// Resolve VCA model parameters from the netlist, with validation.
    ///
    /// 2D current-mode exponential gain: I_sig = G0 * exp(-Vc / VSCALE) * V_sig
    pub(super) fn resolve_vca_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<VcaParams, CodegenError> {
        let vscale = Self::lookup_model_param(netlist, model, "VSCALE").unwrap_or(0.05298);
        let g0 = Self::lookup_model_param(netlist, model, "G0").unwrap_or(1.0);
        let thd = Self::lookup_model_param(netlist, model, "THD").unwrap_or(0.0);

        validate_positive_finite(vscale, "VCA model VSCALE")?;
        validate_positive_finite(g0, "VCA model G0")?;
        if thd < 0.0 || !thd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "VCA model THD must be non-negative and finite, got {thd}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Vca)?;

        Ok(VcaParams { vscale, g0, thd })
    }

    /// Resolve opto/LDR model params. Resolution order per param: explicit
    /// `.model … LDR(RMIN=… …)` value → catalog entry (by model name, e.g.
    /// `.model VTL5C3 LDR()`) → generic default. Mirrors `ldr.rs` semantics.
    pub(super) fn resolve_ldr_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<crate::device_types::LdrParams, CodegenError> {
        let cat = melange_devices::catalog::ldr::lookup(model);
        let r_min = Self::lookup_model_param(netlist, model, "RMIN")
            .or_else(|| cat.map(|c| c.r_min))
            .unwrap_or(75.0);
        let r_max = Self::lookup_model_param(netlist, model, "RMAX")
            .or_else(|| cat.map(|c| c.r_max))
            .unwrap_or(10e6);
        let gamma = Self::lookup_model_param(netlist, model, "GAMMA")
            .or_else(|| cat.map(|c| c.gamma))
            .unwrap_or(0.7);
        let attack_tau = Self::lookup_model_param(netlist, model, "TAU_A")
            .or_else(|| cat.map(|c| c.attack_tau))
            .unwrap_or(0.005);
        let release_tau = Self::lookup_model_param(netlist, model, "TAU_R")
            .or_else(|| cat.map(|c| c.release_tau))
            .unwrap_or(0.2);

        validate_positive_finite(r_min, "LDR model RMIN")?;
        validate_positive_finite(r_max, "LDR model RMAX")?;
        validate_positive_finite(gamma, "LDR model GAMMA")?;
        validate_positive_finite(attack_tau, "LDR model TAU_A")?;
        validate_positive_finite(release_tau, "LDR model TAU_R")?;
        if r_max <= r_min {
            return Err(CodegenError::InvalidConfig(format!(
                "LDR model '{model}': RMAX ({r_max}) must be greater than RMIN ({r_min})"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Ldr)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Ldr,
            "the built-in default LDR",
        );

        Ok(crate::device_types::LdrParams {
            r_min,
            r_max,
            gamma,
            attack_tau,
            release_tau,
        })
    }

    /// Resolve glow-discharge / neon lamp model params (EXPERIMENTAL Phase 0c
    /// Stage 2a). Resolution per param: explicit `.model … NEON(VO=… …)` value
    /// → generic default. No catalog (throwaway experimental device).
    pub(super) fn resolve_glow_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<crate::device_types::GlowParams, CodegenError> {
        // Option-A maintaining-line parameterisation (datasheet-sourced). The
        // lit branch is the affine maintaining line `V(a)−V(k) = v0 + rs·i`.
        // Its intercept `v0` is NOT authored directly — it is derived from the
        // datasheet static maintaining voltage `VM` (measured at the rated
        // current `IK`) and the slope `RS`, so the deck carries datasheet
        // numbers and melange computes the intercept:  v0 = VM − RS·IK.
        // ZA1001 anchors: VM = 93 V @ IK = 1.5 mA; RS ≈ 2.5–4.25 kΩ (ZA1004
        // form-transfer, mid 3 kΩ). The OLD model used VM directly as the
        // intercept (fixed VD = 93), parking the reset floor ~4–6 V too high.
        let vo = Self::lookup_model_param(netlist, model, "VO").unwrap_or(135.0);
        let vm = Self::lookup_model_param(netlist, model, "VM").unwrap_or(93.0);
        let ik = Self::lookup_model_param(netlist, model, "IK").unwrap_or(1.5e-3);
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(3.0e3);
        let roff = Self::lookup_model_param(netlist, model, "ROFF").unwrap_or(300e6);
        // Holding current: the lit→dark extinction threshold on conduction
        // current. Default 2e-4 A (datasheet ZA1004 minimum-sustaining regime).
        // The reset floor lands at v0 + rs·ihold. Must exceed the lit
        // equilibrium sustaining current (Vb−v0)/(Rc+rs) for a relaxation
        // oscillator to extinguish; a physical small-neon value does.
        let ihold = Self::lookup_model_param(netlist, model, "IHOLD").unwrap_or(2e-4);

        // Relaxing-section lit branch (Benson & Bradshaw 1965; defaults-off).
        // RT = DC asymptote of the maintaining line (defaults to RS, so a deck
        // that authors no sections reduces exactly to the static model). K1..K4
        // = delayed-overvoltage coefficients [volts] (default 0 = section off),
        // TAU1..TAU4 = current-lag time constants [seconds]. A section is
        // "active" when its K is non-zero; a zero K disables the section with no
        // special-casing (both the log term and its Jacobian contribution
        // vanish). Never authored directly: the intercept v0 = VM − RS·IK below
        // is unchanged (R3 reconciliation of RS vs RT is a later authoring
        // concern, not this mechanism).
        let r_t = Self::lookup_model_param(netlist, model, "RT").unwrap_or(rs);
        let mut k = [0.0f64; 4];
        let mut tau = [0.0f64; 4];
        for i in 0..4 {
            k[i] = Self::lookup_model_param(netlist, model, &format!("K{}", i + 1)).unwrap_or(0.0);
            tau[i] =
                Self::lookup_model_param(netlist, model, &format!("TAU{}", i + 1)).unwrap_or(0.0);
        }
        let has_sections = k.iter().any(|&x| x != 0.0);

        // IFLOOR (A1): the log-domain current clamp / section-lag seed floor.
        // Authored key; defaults to IHOLD for continuity but is a live edge knob.
        let ifloor = Self::lookup_model_param(netlist, model, "IFLOOR").unwrap_or(ihold);

        // KSUB (part-a): static subnormal-branch slope κ [V per e-fold]; default
        // 0 = off (lit branch bit-identical to the no-KSUB form). Anchored at the
        // rated current IK (where g = VM). Only meaningful WITH sections.
        let ksub = Self::lookup_model_param(netlist, model, "KSUB").unwrap_or(0.0);

        // Ignition depression D(t_off) (Part B; default-off). D_AMP=0 → OFF and
        // no state slot / plain VO strike test (byte-identical). Curve:
        // D = clamp(D_AMP·ln(D_TKNEE/max(t_off, D_THOLD)), 0, VO−VM).
        let d_amp = Self::lookup_model_param(netlist, model, "D_AMP").unwrap_or(0.0);
        let d_tknee = Self::lookup_model_param(netlist, model, "D_TKNEE").unwrap_or(0.0);
        let d_thold = Self::lookup_model_param(netlist, model, "D_THOLD").unwrap_or(0.0);

        validate_positive_finite(vo, "NEON model VO")?;
        validate_positive_finite(vm, "NEON model VM")?;
        validate_positive_finite(ik, "NEON model IK")?;
        validate_positive_finite(rs, "NEON model RS")?;
        validate_positive_finite(roff, "NEON model ROFF")?;
        validate_positive_finite(ihold, "NEON model IHOLD")?;
        // r_t is a DC resistance that may be ~0 (normal glow) but never negative
        // or non-finite. Sections need a positive time constant only where the
        // coefficient is non-zero (an active section); a zero-K section is off
        // and its TAU is ignored.
        if !r_t.is_finite() || r_t < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': RT ({r_t}) must be finite and non-negative"
            )));
        }
        for i in 0..4 {
            if !k[i].is_finite() || !tau[i].is_finite() {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': K{}/TAU{} must be finite",
                    i + 1,
                    i + 1
                )));
            }
            if k[i] != 0.0 && tau[i] <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': TAU{} ({}) must be > 0 when K{} ({}) is non-zero \
                     (active relaxing section needs a positive current-lag time constant)",
                    i + 1,
                    tau[i],
                    i + 1,
                    k[i]
                )));
            }
        }
        validate_positive_finite(ifloor, "NEON model IFLOOR")?;
        // KSUB (subnormal slope) validation. Non-negative; requires sections (the
        // static log term is folded into the section-branch g(I) — a KSUB-only
        // deck would emit the static linear path and silently drop it). The
        // R_T>0 && κ>ΣK corner makes the inner-Newton residual r'(x)=R_T·eˣ+(S−κ)
        // change sign (two roots / none) — reject it; R_T=0 (analog-EE authoring guidance)
        // or κ≤ΣK stays single-signed and globally convergent.
        if !ksub.is_finite() || ksub < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': KSUB ({ksub}) must be finite and non-negative"
            )));
        }
        if ksub != 0.0 {
            if !has_sections {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': KSUB ({ksub}) requires ≥1 active section (K1..K4); the \
                     subnormal term is folded into the relaxing-section lit branch, not the static path"
                )));
            }
            let k_sum: f64 = k.iter().sum();
            if r_t > 0.0 && ksub > k_sum {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': KSUB ({ksub}) > ΣK ({k_sum}) with RT ({r_t}) > 0 is a \
                     non-monotone lit branch (r'(x)=RT·eˣ+(ΣK−KSUB) changes sign → non-convergent); \
                     author RT=0 for a subnormal branch, or keep KSUB ≤ ΣK"
                )));
            }
        }
        // Ignition-depression validation (only meaningful when D_AMP ≠ 0).
        if !d_amp.is_finite() || d_amp < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': D_AMP ({d_amp}) must be finite and non-negative"
            )));
        }
        if d_amp != 0.0 {
            if !(d_tknee > 0.0 && d_tknee.is_finite()) {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': D_TKNEE ({d_tknee}) must be > 0 when D_AMP is non-zero"
                )));
            }
            if !(d_thold > 0.0 && d_thold.is_finite()) {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': D_THOLD ({d_thold}) must be > 0 when D_AMP is non-zero"
                )));
            }
        }

        // Derived maintaining-line intercept (A2). With relaxing sections the
        // lower reset floor comes from the section TAIL (Ī lags falling I →
        // negative overvoltage → cv_extinction < VM), so RS is retired from the
        // intercept and v0 = VM − RT·IK (→ VM at RT≈0). Without sections the
        // historical v0 = VM − RS·IK is kept EXACTLY (byte-identity).
        let v0 = if has_sections {
            vm - r_t * ik
        } else {
            vm - rs * ik
        };
        if v0 <= 0.0 {
            let (slope_name, slope_val) = if has_sections {
                ("RT", r_t)
            } else {
                ("RS", rs)
            };
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': derived maintaining-line intercept v0 = VM − {slope_name}·IK \
                 = {vm} − {slope_val}·{ik} = {v0} is non-positive; check VM/{slope_name}/IK"
            )));
        }
        if vo <= vm {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': VO ({vo}) must be greater than VM ({vm}) \
                 (ignition voltage above the maintaining voltage)"
            )));
        }
        if roff <= rs {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': ROFF ({roff}) must be greater than RS ({rs})"
            )));
        }

        // main replaced warn_unrecognized_params with the stricter
        // check_model_params (unknown keys are a hard error; glow/NEON is a
        // melange-native device, so no recognized-but-unimplemented SPICE keys).
        Self::check_model_params(netlist, model, ModelClass::Glow)?;

        Ok(crate::device_types::GlowParams {
            vo,
            v0,
            rs,
            roff,
            ihold,
            r_t,
            k,
            tau,
            ifloor,
            ksub,
            // Subnormal anchor = rated current IK (g = VM there). Only used when ksub≠0.
            i_n: ik,
            d_amp,
            d_tknee,
            d_thold,
            // Hard cap on the ignition depression: V_s,eff never below VM.
            d_cap: vo - vm,
        })
    }
}
