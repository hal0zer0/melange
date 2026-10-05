//! Device-slot construction from the netlist, with per-unit mismatch.

use super::*;

impl CircuitIR {
    pub fn build_device_info(netlist: &Netlist) -> Result<Vec<DeviceSlot>, CodegenError> {
        Self::build_device_info_with_mna(netlist, None)
    }

    /// Deterministic per-(device, param) jitter draw.
    ///
    /// Hashes the netlist seed together with the device name and the
    /// uppercase param tag into an FNV-64 accumulator, then runs the
    /// SplitMix64 finalizer so well-correlated input bits can't produce
    /// correlated output bits. The high 53 bits become a uniform double
    /// in [0, 1) and the result is mapped to [-1, 1].
    fn mismatch_draw(seed: u64, device_name: &str, param_name: &str) -> f64 {
        const FNV_PRIME: u64 = 0x100000001b3;
        let mut h = seed ^ 0xcbf29ce484222325u64;
        for b in device_name.as_bytes() {
            h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
        }
        // Null byte separator so "Q1" + "SA" can't collide with "Q" + "1SA".
        h = h.wrapping_mul(FNV_PRIME);
        for b in param_name.as_bytes() {
            h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
        }
        // SplitMix64 finalizer
        h = (h ^ (h >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        h = (h ^ (h >> 27)).wrapping_mul(0x94d049bb133111eb);
        h ^= h >> 31;
        let u01 = (h >> 11) as f64 / (1u64 << 53) as f64;
        2.0 * u01 - 1.0
    }

    /// Look up the tolerance for `param_name` on `device_class` across all
    /// `.mismatch` specs in the netlist. Returns 0.0 when the directive is
    /// absent or the param isn't listed.
    fn mismatch_tol_for(netlist: &Netlist, device_class: char, param_name: &str) -> f64 {
        // Unit-variation kill switch (`ParseOptions::disable_unit_variation`,
        // set by `melange validate`). Reporting a zero tolerance here makes
        // `apply_mismatch` a bit-identical pass-through for every device and
        // every parameter, which is the `.mismatch` half of the same switch
        // that skips `.tolerance` in the parser. The specs stay on the netlist
        // so the caller can still name what it disabled.
        if netlist.unit_variation_disabled {
            return 0.0;
        }
        let upper = param_name;
        let mut tol = 0.0f64;
        for spec in &netlist.mismatch_specs {
            if spec.device_class != device_class {
                continue;
            }
            for (k, v) in &spec.params {
                if k == upper {
                    tol = *v; // last one wins (documented behavior)
                }
            }
        }
        tol
    }

    /// Multiply `nominal` by `(1 + tol · u)` where `u ∈ [-1, 1]` is drawn
    /// deterministically from the (seed, device, param) triple. When the
    /// tolerance is zero this is a pure pass-through — the return value
    /// is bit-identical to `nominal`.
    /// A model's thermal resistance `RTH`: infinite (self-heating off) when
    /// the card does not set it, or when the netlist was parsed isothermal
    /// (`ParseOptions::disable_self_heating`, `melange validate`). `TAMB`
    /// keeps its static role either way.
    pub(super) fn resolve_rth(netlist: &Netlist, model: &str) -> f64 {
        if netlist.self_heating_disabled {
            return f64::INFINITY;
        }
        Self::lookup_model_param(netlist, model, "RTH").unwrap_or(f64::INFINITY)
    }

    fn apply_mismatch(
        netlist: &Netlist,
        device_name: &str,
        param_name: &str,
        device_class: char,
        nominal: f64,
    ) -> f64 {
        let tol = Self::mismatch_tol_for(netlist, device_class, param_name);
        if tol == 0.0 {
            return nominal;
        }
        let seed = netlist.seed.unwrap_or(0);
        let u = Self::mismatch_draw(seed, device_name, param_name);
        nominal * (1.0 + tol * u)
    }

    /// Apply per-device `.mismatch T …` jitter to a tube's Koren parameters.
    ///
    /// Pushing mismatch to the *device* params (not the shared `.model` card)
    /// is what makes a push-pull tube pair audibly asymmetric — the dominant
    /// even-harmonic ("H2") source in an otherwise-balanced push-pull stage,
    /// where identical-model halves cancel even harmonics exactly. Shared by
    /// the `Triode` and `Pentode` arms (both carry `TubeParams`). Bit-identical
    /// pass-through when no `.mismatch T` directive lists the param (tol == 0).
    fn apply_tube_mismatch(netlist: &Netlist, name: &str, p: &mut crate::device_types::TubeParams) {
        p.mu = Self::apply_mismatch(netlist, name, "MU", 'T', p.mu);
        p.ex = Self::apply_mismatch(netlist, name, "EX", 'T', p.ex);
        p.kg1 = Self::apply_mismatch(netlist, name, "KG1", 'T', p.kg1);
        p.kp = Self::apply_mismatch(netlist, name, "KP", 'T', p.kp);
        p.kvb = Self::apply_mismatch(netlist, name, "KVB", 'T', p.kvb);
        // Pentode-only screen-current sensitivity; 0.0 on triodes → skip like
        // the DiodeParams.rs guard so triodes stay pure pass-through.
        if p.kg2 > 0.0 {
            p.kg2 = Self::apply_mismatch(netlist, name, "KG2", 'T', p.kg2);
        }
    }

    /// `.mismatch J` strength keys are per channel law: `IDSS` jitters a
    /// LEVEL=1 JFET, `BETA` a LEVEL=2 one. A key no JFET of the deck reads
    /// would jitter nothing, so it is refused, naming the key that would.
    fn check_jfet_mismatch_keys(
        netlist: &Netlist,
        slots: &[DeviceSlot],
    ) -> Result<(), CodegenError> {
        let jfets = slots.iter().filter_map(|s| match &s.params {
            DeviceParams::Jfet(jp) => Some(jp.ps.is_some()),
            _ => None,
        });
        let (mut has_l1, mut has_l2) = (false, false);
        for level2 in jfets {
            if level2 {
                has_l2 = true;
            } else {
                has_l1 = true;
            }
        }
        if !has_l1 && !has_l2 {
            return Ok(());
        }
        for (key, unread, instead) in [
            ("IDSS", !has_l1, "BETA (the LEVEL=2 strength)"),
            ("BETA", !has_l2, "IDSS (the LEVEL=1 strength)"),
        ] {
            if unread && Self::mismatch_tol_for(netlist, 'J', key) != 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    ".mismatch J {key}=: no JFET in this deck has {key} as its strength \
                     parameter, so it would jitter nothing; use {instead}"
                )));
            }
        }
        Ok(())
    }

    /// Build device info, optionally using MNA device dimensions (for forward-active BJTs).
    pub fn build_device_info_with_mna(
        netlist: &Netlist,
        mna: Option<&crate::mna::MnaSystem>,
    ) -> Result<Vec<DeviceSlot>, CodegenError> {
        let mut slots = Vec::new();
        let mut dim_offset = 0;
        let mut nl_dev_idx = 0; // tracks position in mna.nonlinear_devices

        for elem in &netlist.elements {
            match elem {
                Element::Diode { name, model, .. } => {
                    let mut params = Self::resolve_diode_params(netlist, model)?;
                    // Per-diode `.mismatch D …` jitter. No-op when the
                    // directive is absent or the param isn't listed.
                    params.is = Self::apply_mismatch(netlist, name, "IS", 'D', params.is);
                    params.n_vt = Self::apply_mismatch(netlist, name, "N", 'D', params.n_vt);
                    if params.rs > 0.0 {
                        params.rs = Self::apply_mismatch(netlist, name, "RS", 'D', params.rs);
                    }
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                Element::Bjt { name, model, .. } => {
                    // Skip linearized BJTs — they're not in the nonlinear system
                    let is_linearized = mna.is_some_and(|m| {
                        m.linearized_bjts
                            .iter()
                            .any(|l| l.name.eq_ignore_ascii_case(name))
                    });
                    if is_linearized {
                        continue; // Don't create a DeviceSlot, don't increment nl_dev_idx
                    }
                    let mut params = Self::resolve_bjt_params(netlist, model)?;
                    // Per-BJT `.mismatch Q …` jitter. Pushing mismatch to the
                    // *device* params (not the shared `.model`) is what makes
                    // push-pull pairs and antiparallel-style stages audibly
                    // asymmetric even when both transistors point at the same
                    // model card.
                    params.is = Self::apply_mismatch(netlist, name, "IS", 'Q', params.is);
                    params.beta_f = Self::apply_mismatch(netlist, name, "BF", 'Q', params.beta_f);
                    params.beta_r = Self::apply_mismatch(netlist, name, "BR", 'Q', params.beta_r);
                    // Check if MNA has this BJT as forward-active (1D)
                    let is_fa = mna.is_some_and(|m| {
                        nl_dev_idx < m.nonlinear_devices.len()
                            && m.nonlinear_devices[nl_dev_idx].device_type
                                == crate::mna::NonlinearDeviceType::BjtForwardActive
                    });
                    let (dev_type, dim) = if is_fa {
                        (DeviceType::BjtForwardActive, 1)
                    } else {
                        (DeviceType::Bjt, 2)
                    };
                    slots.push(DeviceSlot {
                        device_type: dev_type,
                        start_idx: dim_offset,
                        dimension: dim,
                        params: DeviceParams::Bjt(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Jfet { name, model, .. } => {
                    let mut params = Self::resolve_jfet_params(netlist, model)?;
                    // Per-JFET `.mismatch J …` jitter on the core transfer
                    // parameters. No-op when the directive is absent. A
                    // LEVEL=2 device's strength is BETA, set independently
                    // of VP (IDSS is display only there).
                    if let Some(ps) = params.ps.as_mut() {
                        ps.beta = Self::apply_mismatch(netlist, name, "BETA", 'J', ps.beta);
                    } else {
                        params.idss = Self::apply_mismatch(netlist, name, "IDSS", 'J', params.idss);
                    }
                    params.vp = Self::apply_mismatch(netlist, name, "VP", 'J', params.vp);
                    if let Some(ps) = &params.ps {
                        params.idss = ps.beta * params.vp * params.vp;
                    }
                    params.lambda =
                        Self::apply_mismatch(netlist, name, "LAMBDA", 'J', params.lambda);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Jfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Jfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Triode { name, model, .. } => {
                    // Skip linearized triodes — they're not in the nonlinear system
                    let is_linearized = mna.is_some_and(|m| {
                        m.linearized_triodes
                            .iter()
                            .any(|l| l.name.eq_ignore_ascii_case(name))
                    });
                    if is_linearized {
                        continue; // Don't create a DeviceSlot, don't increment nl_dev_idx
                    }
                    let mut params = Self::resolve_tube_params(netlist, model)?;
                    // Per-triode `.mismatch T …` jitter (see `apply_tube_mismatch`).
                    Self::apply_tube_mismatch(netlist, name, &mut params);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Pentode { name, model, .. } => {
                    let mut params = Self::resolve_pentode_params(netlist, model)?;
                    // Per-pentode `.mismatch T …` jitter (see `apply_tube_mismatch`).
                    // Includes KG2 (screen-current sensitivity) for pentodes.
                    Self::apply_tube_mismatch(netlist, name, &mut params);
                    // Check if MNA has this pentode as grid-off (2D reduced).
                    // Phase 1b: after DC-OP detects Vgk < cutoff, the MNA is
                    // rebuilt via `from_netlist_with_grid_off` which stamps
                    // dimension=2 for the named pentodes. Here we reflect that
                    // back into `TubeParams.kind` so codegen dispatches to
                    // `*_pentode_grid_off` helpers.
                    let is_grid_off = mna.is_some_and(|m| {
                        nl_dev_idx < m.nonlinear_devices.len()
                            && m.nonlinear_devices[nl_dev_idx].device_type
                                == crate::mna::NonlinearDeviceType::Tube
                            && m.nonlinear_devices[nl_dev_idx].dimension == 2
                            && m.nonlinear_devices[nl_dev_idx].nodes.len() >= 4
                    });
                    let dim = if is_grid_off { 2 } else { 3 };
                    if is_grid_off {
                        params.kind = crate::device_types::TubeKind::SharpPentodeGridOff;
                    }
                    let vg2k_frozen = if is_grid_off {
                        mna.and_then(|m| {
                            if nl_dev_idx < m.nonlinear_devices.len() {
                                let v = m.nonlinear_devices[nl_dev_idx].vg2k_frozen;
                                if v.abs() > 1e-15 {
                                    Some(v)
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        })
                        .unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: dim,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen,
                        stateful: None,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Mosfet { name, model, .. } => {
                    let mut params = Self::resolve_mosfet_params(netlist, model)?;
                    // Per-MOSFET `.mismatch M …` jitter on the core transfer
                    // parameters. No-op when the directive is absent.
                    params.kp = Self::apply_mismatch(netlist, name, "KP", 'M', params.kp);
                    params.vt = Self::apply_mismatch(netlist, name, "VT", 'M', params.vt);
                    params.lambda =
                        Self::apply_mismatch(netlist, name, "LAMBDA", 'M', params.lambda);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Mosfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Mosfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Vca { model, .. } => {
                    let params = Self::resolve_vca_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Vca,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Vca(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Ldr { model, .. } => {
                    let params = Self::resolve_ldr_params(netlist, model)?;
                    // The opaque state-block spec needs node indices (1-based,
                    // 0 = ground), which come from the MNA's built node map via
                    // this device's `nonlinear_devices` entry (node_indices =
                    // [r+, r-, ctrl+, ctrl-]). Present for every codegen path
                    // (all pass `Some(mna)`); `None` only in mna-less helper
                    // paths that never emit the stateful device.
                    let stateful = mna.and_then(|m| {
                        m.nonlinear_devices.get(nl_dev_idx).and_then(|d| {
                            if d.device_type == crate::mna::NonlinearDeviceType::Ldr
                                && d.node_indices.len() >= 4
                            {
                                let ni = &d.node_indices;
                                Some(crate::device_types::StatefulSpec {
                                    state_size: 1,
                                    state_seed: vec![params.r_max],
                                    terminal_nodes: ni.clone(),
                                    driving_nodes: vec![ni[2], ni[3]],
                                })
                            } else {
                                None
                            }
                        })
                    });
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Ldr,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Ldr(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                Element::Glow { model, .. } => {
                    let params = Self::resolve_glow_params(netlist, model)?;
                    // Frozen-latch stateful spec: 1-element opaque state block
                    // (0.0 = dark, 1.0 = lit), seeded dark. terminal_nodes and
                    // driving_nodes are both the device's own [a, k] pair — the
                    // strike/extinguish thresholds are evaluated on the terminal
                    // voltage each sample. node_indices = [a, k] from the MNA.
                    let stateful = mna.and_then(|m| {
                        m.nonlinear_devices.get(nl_dev_idx).and_then(|d| {
                            if d.device_type == crate::mna::NonlinearDeviceType::Glow
                                && d.node_indices.len() >= 2
                            {
                                let ni = &d.node_indices;
                                // Fixed state layout, gated by INDEPENDENT flags
                                // so neither-on = today's 1-slot latch,
                                // byte-identical. Trailing slots are appended in
                                // a fixed order AFTER the Ī block so the eval-site
                                // Ī indices ([1..1+N]) never shift:
                                //   [0]              latch (0=dark, 1=lit) — always
                                //   [1..1+N]         Ī_i current-lags      — has_sections
                                //   [1+Nsec]         t_off since-extinction — has_d
                                //   [1+Nsec+has_d]   extinction-armed flag  — has_sections
                                //   [2+Nsec+has_d]   i_conv_prev (prev converged I) — has_sections
                                //   [3+Nsec+has_d]   pending_extinction debounce    — has_sections
                                // Ī_i seeded at IFLOOR (never 0 → no ln(i/0), eval
                                // starts on the R_T line with zero extra drop);
                                // t_off seeded LARGE so a cold device strikes at
                                // full VO (D≈0) until it first extinguishes; the
                                // armed flag seeds 0 (disarmed) — the strike
                                // (re)sets it and the extinction test is gated on
                                // it, so a still-forming discharge below IHOLD is
                                // not extinguished before it has ever sustained.
                                // i_conv_prev/pending seed 0: robust extinction
                                // needs a monotone converged crossing confirmed
                                // over a 1-sample debounce (rejects the near-V0
                                // integrator ring), so a single-sample dip in the
                                // through-current never extinguishes the tube.
                                let has_sec = params.has_sections();
                                let has_d = params.has_d();
                                let nsec = if has_sec {
                                    crate::device_types::GlowParams::MAX_SECTIONS
                                } else {
                                    0
                                };
                                // has_sections trailing slots: armed + i_conv_prev
                                // + pending_extinction = 3.
                                let n_sec_trailing = if has_sec { 3 } else { 0 };
                                let mut seed =
                                    vec![0.0f64; 1 + nsec + usize::from(has_d) + n_sec_trailing];
                                for s in seed.iter_mut().skip(1).take(nsec) {
                                    *s = params.ifloor;
                                }
                                if has_d {
                                    seed[1 + nsec] = crate::device_types::GlowParams::T_OFF_SEED;
                                }
                                // armed / i_conv_prev / pending (indices
                                // 1+nsec+has_d .. +2) stay at 0.0 (disarmed, no
                                // prior current, no pending crossing).
                                let state_size = seed.len();
                                let state_seed = seed;
                                Some(crate::device_types::StatefulSpec {
                                    state_size,
                                    state_seed,
                                    terminal_nodes: ni.clone(),
                                    driving_nodes: vec![ni[0], ni[1]],
                                })
                            } else {
                                None
                            }
                        })
                    });
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Glow,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Glow(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                // An op-amp is a linear VCCS stamped in mna.rs, not a device
                // slot; its card is still checked here like every other
                // class's, so an unknown key is refused rather than warned.
                Element::Opamp { model, .. } => {
                    Self::check_model_params(netlist, model, ModelClass::Opamp)?;
                }
                _ => {}
            }
        }

        // Mark BJTs with MNA-level internal nodes
        if let Some(m) = mna {
            for slot in &mut slots {
                if slot.device_type == DeviceType::Bjt
                    && m.bjt_internal_nodes
                        .iter()
                        .any(|n| n.start_idx == slot.start_idx)
                {
                    slot.has_internal_mna_nodes = true;
                }
            }
        }

        // MOSFET body effect reads V(source) − V(bulk), so the slots need the
        // node indices before ANY consumer evaluates them. Resolving here, not
        // at each call site, is the point: the nodal IR used to solve its DC
        // operating point first and resolve afterwards, so every nodal build
        // baked an operating point without body effect (measured: a
        // choke-loaded common-source stage at V(src) = 1.101 V, the GAMMA=0
        // answer, against ngspice's 0.903 V) and started each render with a
        // transient toward the body-effect bias.
        if let Some(mna) = mna {
            Self::resolve_mosfet_nodes(&mut slots, mna);
        }

        Self::check_jfet_mismatch_keys(netlist, &slots)?;
        Ok(slots)
    }

    /// Resolve MOSFET source/bulk node indices from MNA nonlinear device info.
    ///
    /// Called after `build_device_info` to populate `source_node` and `bulk_node`
    /// fields in MosfetParams, which are needed for body effect (GAMMA/PHI).
    pub(super) fn resolve_mosfet_nodes(slots: &mut [DeviceSlot], mna: &MnaSystem) {
        for slot in slots.iter_mut() {
            if let DeviceParams::Mosfet(ref mut mp) = slot.params {
                if mp.has_body_effect() {
                    // Find the matching MOSFET in MNA nonlinear_devices
                    for dev in &mna.nonlinear_devices {
                        if dev.device_type == crate::mna::NonlinearDeviceType::Mosfet
                            && dev.start_idx == slot.start_idx
                        {
                            // node_indices: [drain, gate, source, bulk]
                            // node_indices are 1-based (0 = ground)
                            // For the N-dimensional system, node index i maps to v[i-1]
                            mp.source_node = dev.node_indices[2];
                            mp.bulk_node = dev.node_indices[3];
                            break;
                        }
                    }
                }
            }
        }
    }
}
