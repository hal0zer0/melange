//! Diode, BJT, JFET and MOSFET `.model` parameter resolution.

use super::*;

impl CircuitIR {
    /// Resolve diode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default.
    pub(super) fn resolve_diode_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<DiodeParams, CodegenError> {
        let vt = melange_primitives::VT_ROOM;
        let cat = melange_devices::catalog::diodes::lookup(model);
        let is = match Self::lookup_model_param(netlist, model, "IS").or_else(|| cat.map(|c| c.is))
        {
            Some(v) => v,
            None => {
                // SPICE default diode (IS=1e-14, N=1.0) for ngspice parity.
                // The old fallback was a chimera: 1N4148's IS (2.52e-9) paired
                // with N=1.0 — matched neither the SPICE default nor a 1N4148.
                crate::diag_warn!(
                    "Diode model '{}' not in catalog and no IS given — falling back to the SPICE default diode (IS=1e-14, N=1.0)",
                    model
                );
                1e-14
            }
        };
        let n = Self::lookup_model_param(netlist, model, "N")
            .or_else(|| cat.map(|c| c.n))
            .unwrap_or(1.0);

        validate_positive_finite(is, "diode model IS")?;
        validate_positive_finite(n, "diode model N")?;

        // Junction capacitance (optional, default 0.0)
        let cjo = Self::lookup_model_param(netlist, model, "CJO").unwrap_or(0.0);
        if cjo < 0.0 || !cjo.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model CJO must be non-negative and finite, got {cjo}"
            )));
        }

        // Series resistance (optional, default 0.0)
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs < 0.0 || !rs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model RS must be non-negative and finite, got {rs}"
            )));
        }

        // Reverse breakdown voltage (optional, default infinity = disabled)
        let bv = Self::lookup_model_param(netlist, model, "BV").unwrap_or(f64::INFINITY);
        if bv.is_finite() {
            validate_positive_finite(bv, "diode model BV")?;
        }

        // Reverse breakdown current (optional; SPICE3f5 default IBV=1e-3).
        // A smaller default shifts the breakdown knee ~0.4·N V past BV.
        let ibv = Self::lookup_model_param(netlist, model, "IBV").unwrap_or(1e-3);
        if ibv.is_finite() {
            validate_positive_finite(ibv, "diode model IBV")?;
        }

        // Self-heating parameters (optional). Defaults match BjtParams so
        // `.model D(RTH=50)` with everything else implicit gives a sensible
        // silicon diode with 1 ms thermal memory.
        let rth = Self::resolve_rth(netlist, model);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }

        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(1e-3);
        if cth < 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if cth == 0.0 {
            log::info!(
                "Diode model '{}': CTH=0 — thermal state has no memory; junction temperature tracks dissipation quasi-statically (Tj = TAMB + RTH·P each sample)",
                model
            );
        }

        let xti = Self::lookup_model_param(netlist, model, "XTI").unwrap_or(3.0);
        if !xti.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model XTI must be finite, got {xti}"
            )));
        }

        let eg = Self::lookup_model_param(netlist, model, "EG").unwrap_or(1.11);
        if eg <= 0.0 || !eg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model EG must be positive and finite, got {eg}"
            )));
        }

        let tamb =
            Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(melange_primitives::T_NOM);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model TAMB must be positive and finite, got {tamb}"
            )));
        }

        // The card is SPICE's, extracted at TNOM; the device sits at TAMB.
        // Scale IS and N·Vt there with the SPICE3 diode law (ngspice
        // `diotemp.c`). Self-heating then moves Tj from TAMB with the same law
        // written relative to TAMB (`emit_self_heating_thermal_updates`); the
        // law composes, so a junction at Tj sees exactly IS(TNOM -> Tj). At
        // TAMB = TNOM every factor is exactly 1.
        let t = tamb / melange_primitives::T_NOM;
        let vt = vt * t;
        let is = is * t.powf(xti / n) * ((t - 1.0) * eg / (n * vt)).exp();
        validate_positive_finite(is, "diode model IS at TAMB")?;

        Self::check_model_params(netlist, model, ModelClass::Diode)?;
        // NOTE: no warn_unresolved_model() here — the diode resolver already
        // emits its own dedicated fallback warning in the IS-resolution arm
        // above ("not in catalog and no IS given — falling back to the SPICE
        // default diode"). Adding the general warning would double-warn.

        Ok(DiodeParams {
            is,
            n_vt: n * vt,
            cjo,
            rs,
            bv,
            ibv,
            rth,
            cth,
            xti,
            eg,
            tamb,
        })
    }

    /// Resolve BJT model parameters from the netlist, with validation.
    ///
    /// Gummel-Poon parameters (VAF, VAR, IKF, IKR) default to infinity,
    /// which collapses qb→1.0, giving exact Ebers-Moll behavior.
    pub(super) fn resolve_bjt_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<BjtParams, CodegenError> {
        let cat = melange_devices::catalog::bjts::lookup(model);
        let vt = Self::lookup_model_param(netlist, model, "VT")
            .or_else(|| cat.map(|c| c.vt))
            .unwrap_or(melange_primitives::VT_ROOM);
        // Card, then catalog part, then the SPICE / ngspice default (IS 1e-16,
        // BF 100, BR 1): a card that omits a parameter means what it means in
        // SPICE.
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(1e-16);
        let beta_f = Self::lookup_model_param(netlist, model, "BF")
            .or_else(|| cat.map(|c| c.beta_f))
            .unwrap_or(100.0);
        let beta_r = Self::lookup_model_param(netlist, model, "BR")
            .or_else(|| cat.map(|c| c.beta_r))
            .unwrap_or(1.0);

        validate_positive_finite(is, "BJT model IS")?;
        validate_positive_finite(vt, "BJT model VT")?;
        validate_positive_finite(beta_f, "BJT model BF")?;
        validate_positive_finite(beta_r, "BJT model BR")?;

        let is_pnp = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
            .unwrap_or(cat.map(|c| c.is_pnp).unwrap_or(false));

        // Gummel-Poon parameters (default to infinity = pure Ebers-Moll)
        let vaf = Self::lookup_model_param(netlist, model, "VAF")
            .or_else(|| Self::lookup_model_param(netlist, model, "VA"))
            .or_else(|| cat.map(|c| c.vaf))
            .unwrap_or(f64::INFINITY);
        let var = Self::lookup_model_param(netlist, model, "VAR")
            .or_else(|| Self::lookup_model_param(netlist, model, "VB"))
            .or_else(|| cat.map(|c| c.var))
            .unwrap_or(f64::INFINITY);
        let ikf = Self::lookup_model_param(netlist, model, "IKF")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBF"))
            .or_else(|| cat.map(|c| c.ikf))
            .unwrap_or(f64::INFINITY);
        let ikr = Self::lookup_model_param(netlist, model, "IKR")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBR"))
            .or_else(|| cat.map(|c| c.ikr))
            .unwrap_or(f64::INFINITY);

        // Validate: if finite, must be positive
        if vaf.is_finite() {
            validate_positive_finite(vaf, "BJT model VAF")?;
        }
        if var.is_finite() {
            validate_positive_finite(var, "BJT model VAR")?;
        }
        if ikf.is_finite() {
            validate_positive_finite(ikf, "BJT model IKF")?;
        }
        if ikr.is_finite() {
            validate_positive_finite(ikr, "BJT model IKR")?;
        }

        // Junction capacitances (optional, default 0.0)
        let cje = Self::lookup_model_param(netlist, model, "CJE").unwrap_or(0.0);
        if cje < 0.0 || !cje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJE must be non-negative and finite, got {cje}"
            )));
        }
        let cjc = Self::lookup_model_param(netlist, model, "CJC").unwrap_or(0.0);
        if cjc < 0.0 || !cjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJC must be non-negative and finite, got {cjc}"
            )));
        }

        // Depletion-cap parameters (SPICE defaults: VJ = 0.75 V, MJ = 0.33, FC = 0.5).
        // VJ must be strictly positive so `(1 - V/VJ)` is well-defined.
        // MJ is typically in [0.2, 0.5]; we accept anything finite and non-negative.
        // FC must be in [0, 0.95] — values at or above 1 would put the tangent
        // extension inside the singular region of the depletion formula.
        let vje = Self::lookup_model_param(netlist, model, "VJE").unwrap_or(0.75);
        if vje <= 0.0 || !vje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model VJE must be positive and finite, got {vje}"
            )));
        }
        let mje = Self::lookup_model_param(netlist, model, "MJE").unwrap_or(0.33);
        if mje < 0.0 || !mje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model MJE must be non-negative and finite, got {mje}"
            )));
        }
        let vjc = Self::lookup_model_param(netlist, model, "VJC").unwrap_or(0.75);
        if vjc <= 0.0 || !vjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model VJC must be positive and finite, got {vjc}"
            )));
        }
        let mjc = Self::lookup_model_param(netlist, model, "MJC").unwrap_or(0.33);
        if mjc < 0.0 || !mjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model MJC must be non-negative and finite, got {mjc}"
            )));
        }
        let fc = Self::lookup_model_param(netlist, model, "FC").unwrap_or(0.5);
        if !fc.is_finite() || !(0.0..=0.95).contains(&fc) {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model FC must be in [0.0, 0.95], got {fc}"
            )));
        }

        // Forward transit time for diffusion capacitance (default 0 = disabled).
        let tf = Self::lookup_model_param(netlist, model, "TF").unwrap_or(0.0);
        if tf < 0.0 || !tf.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TF must be non-negative and finite, got {tf}"
            )));
        }

        // Forward emission coefficient (default 1.0 = ideal)
        let nf = Self::lookup_model_param(netlist, model, "NF").unwrap_or(1.0);
        if nf <= 0.0 || !nf.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NF must be positive and finite, got {nf}"
            )));
        }

        // B-E leakage saturation current (default 0.0 = disabled)
        let ise = Self::lookup_model_param(netlist, model, "ISE").unwrap_or(0.0);
        if ise < 0.0 || !ise.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISE must be non-negative and finite, got {ise}"
            )));
        }

        // B-E leakage emission coefficient (default 1.5)
        let ne = Self::lookup_model_param(netlist, model, "NE").unwrap_or(1.5);
        if ne <= 0.0 || !ne.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NE must be positive and finite, got {ne}"
            )));
        }

        // Reverse emission coefficient (default 1.0 = ideal)
        let nr = Self::lookup_model_param(netlist, model, "NR").unwrap_or(1.0);
        if nr <= 0.0 || !nr.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NR must be positive and finite, got {nr}"
            )));
        }

        // B-C leakage saturation current (default 0.0 = disabled)
        let isc = Self::lookup_model_param(netlist, model, "ISC").unwrap_or(0.0);
        if isc < 0.0 || !isc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISC must be non-negative and finite, got {isc}"
            )));
        }

        // B-C leakage emission coefficient (default 2.0)
        let nc = Self::lookup_model_param(netlist, model, "NC").unwrap_or(2.0);
        if nc <= 0.0 || !nc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NC must be positive and finite, got {nc}"
            )));
        }

        // Parasitic series resistances (optional, default 0.0)
        let rb = Self::lookup_model_param(netlist, model, "RB").unwrap_or(0.0);
        let rc = Self::lookup_model_param(netlist, model, "RC").unwrap_or(0.0);
        let re = Self::lookup_model_param(netlist, model, "RE").unwrap_or(0.0);
        if rb < 0.0 || !rb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RB must be non-negative and finite, got {rb}"
            )));
        }
        if rc < 0.0 || !rc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RC must be non-negative and finite, got {rc}"
            )));
        }
        if re < 0.0 || !re.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RE must be non-negative and finite, got {re}"
            )));
        }

        // Self-heating parameters (optional)
        let rth = Self::resolve_rth(netlist, model);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }

        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(1e-3);
        if cth < 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if cth == 0.0 {
            log::info!(
                "BJT model '{}': CTH=0 — thermal state has no memory; junction temperature tracks dissipation quasi-statically (Tj = TAMB + RTH·P each sample)",
                model
            );
        }

        // SPICE `XTB`: forward/reverse beta temperature exponent, used by the
        // self-heating block as `BF(T) = BF·(Tj/Tnom)^XTB` (likewise `BR`).
        // Default 0.0 is SPICE's own, and makes the power term exactly 1.0 —
        // so a card without `XTB`, and any card at all when `Tj == Tnom`,
        // leaves beta untouched and the emitted DSP byte-identical.
        let xtb = Self::lookup_model_param(netlist, model, "XTB").unwrap_or(0.0);
        if !xtb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model XTB must be finite, got {xtb}"
            )));
        }
        let xti = Self::lookup_model_param(netlist, model, "XTI").unwrap_or(3.0);
        if !xti.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model XTI must be finite, got {xti}"
            )));
        }

        let eg = Self::lookup_model_param(netlist, model, "EG").unwrap_or(1.11);
        if eg <= 0.0 || !eg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model EG must be positive and finite, got {eg}"
            )));
        }

        let tamb =
            Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(melange_primitives::T_NOM);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TAMB must be positive and finite, got {tamb}"
            )));
        }

        // The card is SPICE's, extracted at TNOM; the device sits at TAMB.
        // Scale it there with the SPICE3 BJT law (ngspice `bjttemp.c`): IS
        // through XTI and EG, BF/BR through XTB, and the leakage currents
        // ISE/ISC through both. Self-heating then moves Tj from TAMB with the
        // IS/BF/BR law written relative to TAMB; the law composes. At TAMB =
        // TNOM every factor is exactly 1.
        let t = tamb / melange_primitives::T_NOM;
        let vt = vt * t;
        let factlog = (t - 1.0) * eg / vt + xti * t.ln();
        let bfactor = t.powf(xtb);
        let is = is * factlog.exp();
        let beta_f = beta_f * bfactor;
        let beta_r = beta_r * bfactor;
        let ise = ise * (factlog / ne).exp() / bfactor;
        let isc = isc * (factlog / nc).exp() / bfactor;
        validate_positive_finite(is, "BJT model IS at TAMB")?;

        Self::check_model_params(netlist, model, ModelClass::Bjt)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Bjt,
            "the built-in default BJT",
        );

        Ok(BjtParams {
            is,
            vt,
            beta_f,
            beta_r,
            is_pnp,
            vaf,
            var,
            ikf,
            ikr,
            cje,
            cjc,
            tf,
            vje,
            mje,
            vjc,
            mjc,
            fc,
            nf,
            nr,
            ise,
            ne,
            isc,
            nc,
            rb,
            rc,
            re,
            rth,
            cth,
            xti,
            xtb,
            eg,
            tamb,
        })
    }

    /// Resolve JFET model parameters from the netlist, with validation.
    ///
    /// 2D Shichman-Hodges: IDSS, VP, and LAMBDA control triode + saturation regions.
    pub(super) fn resolve_jfet_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<JfetParams, CodegenError> {
        let cat = melange_devices::catalog::jfets::lookup(model);

        // Determine channel type first — default VP depends on polarity.
        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let default_vp = cat
            .map(|c| c.vp)
            .unwrap_or(if is_p_channel { 2.0 } else { -2.0 });
        // Melange's JFET device convention stores VP POSITIVE for P-channel
        // (the device model flips it internally: vp_eff = -vp for P). Every
        // vendor PJF card uses the SPICE convention VTO < 0, so copying VTO
        // verbatim would double-flip the pinch-off and leave the device dead.
        // Normalize SPICE-convention PJF cards (VTO < 0) to the melange
        // convention (vp = -VTO). N-channel VTO < 0 already matches — unchanged.
        let vp = match Self::lookup_model_param(netlist, model, "VTO") {
            Some(raw_vto) if is_p_channel && raw_vto < 0.0 => {
                crate::diag_warn!(
                    "P-channel JFET model '{}': SPICE-convention VTO={} normalized to melange convention vp={} (P-channel pinch-off stored positive; device model flips internally)",
                    model,
                    raw_vto,
                    -raw_vto
                );
                -raw_vto
            }
            Some(raw_vto) if is_p_channel => {
                log::info!(
                    "P-channel JFET model '{}': VTO={} > 0 accepted as already melange-convention (positive P-channel pinch-off)",
                    model,
                    raw_vto
                );
                raw_vto
            }
            Some(raw_vto) => raw_vto,
            None => default_vp,
        };
        // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2.
        // Uses the normalized vp — sign-safe regardless (vp is squared).
        let idss = if let Some(raw_idss) = Self::lookup_model_param(netlist, model, "IDSS") {
            raw_idss
        } else if let Some(beta) = Self::lookup_model_param(netlist, model, "BETA") {
            beta * vp * vp
        } else {
            // SPICE / ngspice default BETA = 1e-4 A/V^2 (IDSS = BETA * VTO^2).
            cat.map(|c| c.idss).unwrap_or(1e-4 * vp * vp)
        };
        // SPICE / ngspice default LAMBDA = 0.
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        validate_positive_finite(idss, "JFET model IDSS")?;
        if !vp.is_finite() || vp.abs() < 1e-15 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model VP must be finite and nonzero, got {vp}"
            )));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Gate junctions (SPICE IS, N; ngspice defaults). IS = 0 disables them.
        let is = Self::lookup_model_param(netlist, model, "IS").unwrap_or(1e-14);
        if is < 0.0 || !is.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model IS must be non-negative and finite, got {is}"
            )));
        }
        let n = Self::lookup_model_param(netlist, model, "N").unwrap_or(1.0);
        if n <= 0.0 || !n.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model N must be positive and finite, got {n}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Jfet)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Jfet,
            "the built-in default JFET",
        );

        Ok(JfetParams {
            idss,
            vp,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            is,
            n,
        })
    }

    /// Resolve MOSFET model parameters from the netlist, with validation.
    pub(super) fn resolve_mosfet_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<MosfetParams, CodegenError> {
        let cat = melange_devices::catalog::mosfets::lookup(model);

        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PM"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        // Card, then catalog part, then the SPICE / ngspice level-1 default
        // (KP 2e-5 A/V^2 with W = L, VTO 0, LAMBDA 0).
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(2e-5);
        let default_vt = cat.map(|c| c.vt).unwrap_or(0.0);
        let vt = Self::lookup_model_param(netlist, model, "VTO")
            .or_else(|| Self::lookup_model_param(netlist, model, "VT"))
            .unwrap_or(default_vt);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        validate_positive_finite(kp, "MOSFET model KP")?;
        if !vt.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model VT must be finite, got {vt}"
            )));
        }
        // MOSFET VTO sign is PRESERVED (unlike the P-JFET normalization above):
        // the device math honors signed VTO, so NMOS with VTO < 0 is a valid
        // depletion-mode device (conducting at Vgs=0), not a convention clash.
        if !is_p_channel && vt < 0.0 {
            log::info!(
                "NMOS model '{}': VTO={} < 0 — depletion-mode device (conducts at Vgs=0); sign preserved",
                model,
                vt
            );
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Body effect parameters (optional, default 0.0 = disabled)
        let gamma = Self::lookup_model_param(netlist, model, "GAMMA").unwrap_or(0.0);
        let phi = Self::lookup_model_param(netlist, model, "PHI").unwrap_or(0.6);
        if gamma < 0.0 || !gamma.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model GAMMA must be non-negative and finite, got {gamma}"
            )));
        }
        if phi <= 0.0 || !phi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model PHI must be positive and finite, got {phi}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Mosfet)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Mosfet,
            "the built-in default MOSFET",
        );

        // source_node and bulk_node will be resolved later from the MNA system
        Ok(MosfetParams {
            kp,
            vt,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            gamma,
            phi,
            source_node: 0,
            bulk_node: 0,
        })
    }
}
