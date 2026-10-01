//! Triode and pentode `.model` parameter resolution.

use super::*;

impl CircuitIR {
    /// Report the **grid current starting point** this triode's fitted grid law
    /// implies, and warn when it falls outside the manufacturer's per-type
    /// limit for that tube.
    ///
    /// This is a CHECK, deliberately not a parameter: a parameter would invite
    /// someone to fit it, and the whole value of the onset is that it is an
    /// *output* of `(Gg, xi, Cg)` that an independent datasheet row can score.
    ///
    /// The criterion is the manufacturers' own: `Ig = +0.3 µA`, positive, into
    /// the grid (Philips ECC82 1959 footnote, spelled out in full there; the
    /// ECC83 sheet carries the same row). The limit is read per tube type from
    /// that type's own sheet — ECC83 `max -0.9 V`, ECC82 `max -1.3 V` — never
    /// from one global constant, because the types genuinely differ. A type
    /// with no limit on file is reported and not checked.
    fn report_grid_start_point(model: &str, in_catalog: bool, gg: f64, xi: f64, cg: f64) {
        let tube = melange_devices::KorenTriode {
            mu: 100.0,
            ex: 1.4,
            kg1: 1060.0,
            kp: 600.0,
            kvb: 300.0,
            gg,
            xi,
            cg,
            lambda: 0.0,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        };
        let Some(onset) =
            tube.grid_voltage_at_current(melange_devices::tube::GRID_START_CRITERION_A)
        else {
            crate::diag_warn!(
                "Triode '{model}': grid law (Gg={gg:.4e}, xi={xi}, Cg={cg}) never reaches the \
                 0.3 uA grid-current starting point — the onset check cannot be evaluated."
            );
            return;
        };
        let ig_at_zero = tube.grid_current(0.0);
        let provenance = if in_catalog {
            ""
        } else {
            " [no catalog entry: grid law is the shipped 12AX7 default unless the deck set \
             GG/XI/CG]"
        };
        log::info!(
            "Triode '{model}': grid current starts (Ig = +0.3 uA) at Vgk = {onset:.3} V; \
             Ig(0 V) = {:.2} uA{provenance}",
            ig_at_zero * 1e6
        );
        match melange_devices::catalog::tubes::grid_start_limit_v(model) {
            Some(limit) if onset < limit => crate::diag_warn!(
                "Triode '{model}': derived grid-current starting point {onset:.3} V is BELOW the \
                 manufacturer limit for this type (Vg(Ig = +0.3 uA) max {limit:.1} V). The fitted \
                 grid law conducts further into the negative-grid region than the type is \
                 specified to."
            ),
            Some(limit) if onset >= 0.0 => crate::diag_warn!(
                "Triode '{model}': derived grid-current starting point {onset:.3} V is at or \
                 above 0 V, so this grid law has no negative-grid conduction at the 0.3 uA \
                 criterion at all. Every measured 12AX7 starts between -0.27 and -0.38 V, and \
                 the type's own limit is {limit:.1} V. Check GG/XI/CG."
            ),
            Some(_) => {}
            None => log::info!(
                "Triode '{model}': no manufacturer grid-current starting-point limit on file for \
                 this type — onset reported, not checked."
            ),
        }
    }

    /// Resolve tube/triode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default (12AX7).
    pub(super) fn resolve_tube_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<TubeParams, CodegenError> {
        let cat = melange_devices::catalog::tubes::lookup(model);
        let mu = Self::lookup_model_param(netlist, model, "MU")
            .or_else(|| cat.map(|c| c.mu))
            .unwrap_or(100.0);
        let ex = Self::lookup_model_param(netlist, model, "EX")
            .or_else(|| cat.map(|c| c.ex))
            .unwrap_or(1.4);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1")
            .or_else(|| cat.map(|c| c.kg1))
            .unwrap_or(1060.0);
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(600.0);
        let kvb = Self::lookup_model_param(netlist, model, "KVB")
            .or_else(|| cat.map(|c| c.kvb))
            .unwrap_or(300.0);
        // Dempwolf & Zölzer DAFx-11 eq. (11) grid law. `IG_MAX`/`VGK_ONSET` are
        // RETIRED, not aliased and not repurposed — `check_model_params` below
        // refuses either key and prints the conversion. See `model_params.rs`.
        let gg = Self::lookup_model_param(netlist, model, "GG")
            .or_else(|| cat.map(|c| c.gg))
            .unwrap_or(melange_devices::tube::DEFAULT_GG);
        let xi = Self::lookup_model_param(netlist, model, "XI")
            .or_else(|| cat.map(|c| c.xi))
            .unwrap_or(melange_devices::tube::DEFAULT_XI);
        let cg = Self::lookup_model_param(netlist, model, "CG")
            .or_else(|| cat.map(|c| c.cg))
            .unwrap_or(melange_devices::tube::DEFAULT_CG);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        // The triode is sharp-cutoff: its DC operating point and transient
        // evaluate the single-section Koren law. A card's MU_B/SVAR/EX_B is
        // refused (model_params TRIODE_REFUSED), and the fields stay 0, so every
        // estimate built from these params is of the tube as built.
        let (mu_b, svar, ex_b) = (0.0, 0.0, 0.0);

        validate_positive_finite(mu, "tube model MU")?;
        validate_positive_finite(ex, "tube model EX")?;
        validate_positive_finite(kg1, "tube model KG1")?;
        validate_positive_finite(kp, "tube model KP")?;
        validate_positive_finite(kvb, "tube model KVB")?;
        validate_positive_finite(gg, "tube model GG")?;
        validate_positive_finite(xi, "tube model XI")?;
        validate_positive_finite(cg, "tube model CG")?;

        // Validate optional lambda: must be non-negative and finite
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Inter-electrode capacitances (optional, default 0.0)
        let ccg = Self::lookup_model_param(netlist, model, "CCG").unwrap_or(0.0);
        if ccg < 0.0 || !ccg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCG must be non-negative and finite, got {ccg}"
            )));
        }
        let cgp = Self::lookup_model_param(netlist, model, "CGP").unwrap_or(0.0);
        if cgp < 0.0 || !cgp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CGP must be non-negative and finite, got {cgp}"
            )));
        }
        let ccp = Self::lookup_model_param(netlist, model, "CCP").unwrap_or(0.0);
        if ccp < 0.0 || !ccp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCP must be non-negative and finite, got {ccp}"
            )));
        }

        // Grid internal resistance (optional, default 0.0 = disabled)
        let rgi = Self::lookup_model_param(netlist, model, "RGI").unwrap_or(0.0);
        if rgi < 0.0 || !rgi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model RGI must be non-negative and finite, got {rgi}"
            )));
        }

        // Self-heating thermal params (optional; disabled by default so the
        // generated solver is byte-identical for every non-thermal triode).
        // Mirrors the BJT/diode contract: `RTH` is the gate — any finite,
        // positive value activates the per-sample envelope-temperature update
        // and the `VBIAS_ALPHA · (Tp - TAMB)` Vgk drift baked into the Koren
        // call site. `CTH` sets the thermal time constant τ = RTH·CTH. Pentode
        // support is gated at the model layer (`has_self_heating`) — the
        // resolver still accepts the params so pentode circuits don't trip
        // the unrecognized-param warning, but the emitter won't use them
        // until pentode screen-dissipation is wired.
        let rth = Self::resolve_rth(netlist, model);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }
        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(0.0);
        if !cth.is_finite() || cth < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if rth.is_finite() && cth == 0.0 {
            log::info!(
                "Tube model '{}': CTH=0 with finite RTH — thermal time constant τ = RTH·CTH is zero; envelope temperature tracks dissipation quasi-statically (Tp = TAMB + RTH·P each sample)",
                model
            );
        }
        let vbias_alpha = Self::lookup_model_param(netlist, model, "VBIAS_ALPHA").unwrap_or(0.0);
        if !vbias_alpha.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model VBIAS_ALPHA must be finite, got {vbias_alpha}"
            )));
        }
        let tamb = Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(300.15);
        if !tamb.is_finite() || tamb <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model TAMB must be positive and finite, got {tamb}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Triode)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Triode,
            "a default 12AX7-class triode",
        );

        Self::report_grid_start_point(model, cat.is_some(), gg, xi, cg);

        Ok(TubeParams {
            kind: crate::device_types::TubeKind::SharpTriode,
            mu,
            ex,
            kg1,
            kp,
            kvb,
            // Leach fields, unused by the triode path (see `TubeParams::ig_max`).
            ig_max: 0.0,
            vgk_onset: 0.0,
            gg,
            xi,
            cg,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2: 0.0,
            alpha_s: 0.0,
            a_factor: 0.0,
            beta_factor: 0.0,
            // Phase 5: partition noise is pentode-only. Triode plate-shot is
            // bare Schottky `2q·Ip`; PARTITION_F is unused and the field is
            // gated by `is_pentode()` at the codegen-collector layer.
            partition_f: 1.0,
            screen_form: crate::device_types::ScreenForm::Rational,
            mu_b,
            svar,
            ex_b,
            rth,
            cth,
            vbias_alpha,
            tamb,
        })
    }

    /// Resolve pentode model parameters from the netlist, with validation.
    ///
    /// Uses Reefman's pentode equations (see `pentode_equations.md` memory
    /// file). Reads MU, EX, KG1, KG2, KP, KVB, ALPHA_S, A_FACTOR, BETA_FACTOR,
    /// SCREEN_FORM plus the shared triode-compatible params (IG_MAX, VGK_ONSET,
    /// CCG/CGP/CCP, RGI).
    ///
    /// Resolution order (each parameter independently):
    ///   1. Explicit `.model NAME VP(PARAM=value)` in the netlist
    ///   2. `PENTODE_CATALOG` entry keyed by the model name (e.g. `EL84-P`,
    ///      `6L6GC-T`)
    ///   3. Generic EL84-shaped fallback default (lets a bare `.model FOO VP()`
    ///      still produce a working — if wrong — pentode so codegen doesn't
    ///      crash on unfitted circuits)
    ///
    /// The screen-current form (`Rational` / `Exponential`) resolves the same
    /// way: explicit `SCREEN_FORM=0|1` param overrides catalog, catalog
    /// provides the right default for fitted tubes (EL84/EL34/EF86 →
    /// Rational, 6L6GC/6V6GT → Exponential), and the fallback is `Rational`.
    ///
    /// Returns a `TubeParams` with `kind = SharpPentode`. Callers should
    /// `validate()` the result; this function does explicit `Err` on missing
    /// required pentode params (KG2, ALPHA_S).
    pub(super) fn resolve_pentode_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<TubeParams, CodegenError> {
        // Catalog lookup first — if the model name matches a PentodeCatalogEntry,
        // we use those fitted params as the fallback. Explicit `.model VP(...)`
        // parameters override on a per-field basis (user can replace any subset).
        let cat = melange_devices::catalog::tubes::lookup_pentode(model);

        let mu = Self::lookup_model_param(netlist, model, "MU")
            .or_else(|| cat.map(|c| c.mu))
            .unwrap_or(23.36);
        let ex = Self::lookup_model_param(netlist, model, "EX")
            .or_else(|| cat.map(|c| c.ex))
            .unwrap_or(1.138);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1")
            .or_else(|| cat.map(|c| c.kg1))
            .unwrap_or(117.4);
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(152.4);
        let kvb = Self::lookup_model_param(netlist, model, "KVB")
            .or_else(|| cat.map(|c| c.kvb))
            .unwrap_or(4015.8);
        let kg2 = Self::lookup_model_param(netlist, model, "KG2")
            .or_else(|| cat.map(|c| c.kg2))
            .unwrap_or(1275.0);
        let alpha_s = Self::lookup_model_param(netlist, model, "ALPHA_S")
            .or_else(|| cat.map(|c| c.alpha_s))
            .unwrap_or(7.66);
        // `A` alone collides with other SPICE conventions (e.g. AC), so the
        // model directive uses the more explicit name `A_FACTOR`.
        let a_factor = Self::lookup_model_param(netlist, model, "A_FACTOR")
            .or_else(|| cat.map(|c| c.a_factor))
            .unwrap_or(4.344e-4);
        let beta_factor = Self::lookup_model_param(netlist, model, "BETA_FACTOR")
            .or_else(|| cat.map(|c| c.beta_factor))
            .unwrap_or(0.148);
        // Phase 5 partition-noise multiplier. Default 1.0 (textbook Schottky
        // partition statistics). Not in the catalog — it's a process-variation
        // knob applied at codegen, not a fitted device parameter.
        let partition_f = Self::lookup_model_param(netlist, model, "PARTITION_F").unwrap_or(1.0);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(8e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(0.7);
        // The pentode plate law has no lambda term: a card's LAMBDA is refused
        // (model_params PENTODE_REFUSED), and the field stays 0.
        let lambda = 0.0;

        // Reefman §5 variable-mu (remote-cutoff) parameters. Resolution order
        // matches every other field: explicit `.model` > catalog > default 0.0.
        let mu_b = Self::lookup_model_param(netlist, model, "MU_B")
            .or_else(|| cat.map(|c| c.mu_b))
            .unwrap_or(0.0);
        let svar = Self::lookup_model_param(netlist, model, "SVAR")
            .or_else(|| cat.map(|c| c.svar))
            .unwrap_or(0.0);
        let ex_b = Self::lookup_model_param(netlist, model, "EX_B")
            .or_else(|| cat.map(|c| c.ex_b))
            .unwrap_or(0.0);

        // Screen form: catalog value wins over the default; explicit
        // `SCREEN_FORM=0|1|2` in the .model directive wins over the catalog.
        //   0 = Rational   (Derk §4.4)
        //   1 = Exponential (DerkE §4.5)
        //   2 = Classical   (Norman Koren 1996 / Cohen-Hélie 2010)
        let screen_form = {
            use crate::device_types::ScreenForm;
            let explicit = Self::lookup_model_param(netlist, model, "SCREEN_FORM");
            match explicit {
                Some(v) if v == 0.0 => ScreenForm::Rational,
                Some(v) if v == 1.0 => ScreenForm::Exponential,
                Some(v) if v == 2.0 => ScreenForm::Classical,
                Some(v) => {
                    return Err(CodegenError::InvalidConfig(format!(
                        "pentode model SCREEN_FORM must be 0 (Rational), \
                         1 (Exponential), or 2 (Classical), got {v}"
                    )));
                }
                None => match cat.map(|c| c.screen_form) {
                    Some(melange_devices::tube::ScreenForm::Exponential) => ScreenForm::Exponential,
                    Some(melange_devices::tube::ScreenForm::Classical) => ScreenForm::Classical,
                    _ => ScreenForm::Rational,
                },
            }
        };

        validate_positive_finite(mu, "pentode model MU")?;
        validate_positive_finite(ex, "pentode model EX")?;
        validate_positive_finite(kg1, "pentode model KG1")?;
        validate_positive_finite(kg2, "pentode model KG2")?;
        validate_positive_finite(kp, "pentode model KP")?;
        validate_positive_finite(kvb, "pentode model KVB")?;
        validate_positive_finite(ig_max, "pentode model IG_MAX")?;
        validate_positive_finite(vgk_onset, "pentode model VGK_ONSET")?;

        // Classical Koren does not use alpha_s / a_factor / beta_factor at
        // all — they're ignored by the `*_pentode_classical` helpers. Skip
        // the Derk-specific invariants when the screen form is Classical.
        let uses_derk_shape = !matches!(screen_form, crate::device_types::ScreenForm::Classical);
        if uses_derk_shape {
            validate_positive_finite(alpha_s, "pentode model ALPHA_S")?;
            if !a_factor.is_finite() || a_factor < 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "pentode model A_FACTOR must be non-negative and finite, got {a_factor}"
                )));
            }
            if !beta_factor.is_finite() || beta_factor < 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "pentode model BETA_FACTOR must be non-negative and finite, got {beta_factor}"
                )));
            }
        }
        // Reefman §5 variable-mu constraints (mirrors `TubeParams::validate()`).
        // Surfacing them at the resolver level gives a clearer error site than
        // the downstream `params.validate()` call.
        if !svar.is_finite() || !(0.0..=1.0).contains(&svar) {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model SVAR must be in [0, 1] and finite, got {svar}"
            )));
        }
        if svar > 0.0 {
            if !mu_b.is_finite() || mu_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu pentode MU_B must be positive and finite when SVAR>0, got {mu_b}"
                )));
            }
            if !ex_b.is_finite() || ex_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu pentode EX_B must be positive and finite when SVAR>0, got {ex_b}"
                )));
            }
            // Variable-mu + Classical is unsupported — Reefman §5 is built on
            // the Derk softplus structure, not the Classical arctan knee.
            if matches!(screen_form, crate::device_types::ScreenForm::Classical) {
                return Err(CodegenError::InvalidConfig(
                    "variable-mu Classical Koren pentodes are not implemented; \
                     use SCREEN_FORM=0 (Rational) for variable-mu tubes (6K7/EF89 pattern)"
                        .to_string(),
                ));
            }
        }

        let ccg = Self::lookup_model_param(netlist, model, "CCG").unwrap_or(0.0);
        if ccg < 0.0 || !ccg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CCG must be non-negative and finite, got {ccg}"
            )));
        }
        let cgp = Self::lookup_model_param(netlist, model, "CGP").unwrap_or(0.0);
        if cgp < 0.0 || !cgp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CGP must be non-negative and finite, got {cgp}"
            )));
        }
        let ccp = Self::lookup_model_param(netlist, model, "CCP").unwrap_or(0.0);
        if ccp < 0.0 || !ccp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CCP must be non-negative and finite, got {ccp}"
            )));
        }
        // The pentode has no internal-grid solve: a card's RGI is refused
        // (model_params PENTODE_REFUSED), and the field stays 0.
        let rgi = 0.0;

        Self::check_model_params(netlist, model, ModelClass::Pentode)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Pentode,
            "a default EL84-class pentode",
        );

        let params = TubeParams {
            kind: crate::device_types::TubeKind::SharpPentode,
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            // D&Z triode grid fields, unused on the pentode path: a pentode's
            // control grid keeps the Leach law above (no published D&Z-form fit
            // exists for a power pentode, and melange does not invent one).
            gg: melange_devices::tube::DEFAULT_GG,
            xi: melange_devices::tube::DEFAULT_XI,
            cg: melange_devices::tube::DEFAULT_CG,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2,
            alpha_s,
            a_factor,
            beta_factor,
            partition_f,
            screen_form,
            // Phase 1c variable-mu §5 params, resolved above from the `.model`
            // directive (explicit > catalog > default 0.0) and already
            // validated. Previously these were hardcoded to 0.0, which silently
            // discarded a variable-mu pentode card AFTER it passed validation —
            // the deck compiled as a sharp pentode with no diagnostic. For a
            // sharp card svar/mu_b/ex_b resolve to 0.0, so this is byte-identical
            // for every non-variable-mu pentode; it only changes svar>0 decks
            // (e.g. the 6K7 remote-cutoff stage).
            mu_b,
            svar,
            ex_b,
            // Pentode self-heating not wired yet — screen dissipation needs a
            // separate term (Ip·Vpk + Ig2·Vg2k). Triode path is live.
            rth: f64::INFINITY,
            cth: 0.0,
            vbias_alpha: 0.0,
            tamb: 300.15,
        };
        params.validate().map_err(CodegenError::InvalidConfig)?;
        Ok(params)
    }
}
