//! Forward-active BJT and grid-off pentode reduction detectors.

use super::*;

impl CircuitIR {
    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is non-positive or non-finite.
    /// Detect BJTs that are forward-active at the DC operating point.
    ///
    /// Returns the names (uppercased) of BJTs with Vbc < -0.5V that can be
    /// modeled as 1D (Vbe→Ic only), reducing M by 1 each.
    ///
    /// Only pure Ebers-Moll devices qualify (no Gummel-Poon VAF/VAR/IKF/IKR,
    /// no ISE leakage, no self-heating RTH, no ohmic parasitics RB/RC/RE) —
    /// for those the reduction is exact. GP/leaky BJTs are left full-2D even
    /// when forward-active, because the 1D emission has no qb and uses
    /// Ib = Ic/BF. Self-heating BJTs stay 2D because the thermal update reads
    /// the (Ic, Ib) slot pair at (s, s+1). Parasitic-carded BJTs stay 2D
    /// because the FA emission drops RB/RC/RE entirely (gm·RE is O(1) on
    /// power BJTs).
    ///
    /// Call this BEFORE building the final MNA/kernel. If non-empty, rebuild
    /// MNA with `from_netlist_forward_active()` and kernel before calling `from_kernel()`.
    pub fn detect_forward_active_bjts(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> std::collections::HashSet<String> {
        use crate::codegen::BjtFaMode;
        use crate::dc_op;

        // Netlist-shaped slots (this runs before any FA reduction), with the
        // MOSFET body-effect nodes resolved so this DC OP matches the others.
        let mut device_slots = Self::build_device_info(netlist).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashSet::new();
        }
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        let dc_result =
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config(mna, config));

        let mut forward_active = std::collections::HashSet::new();
        for (slot_idx, slot) in device_slots.iter().enumerate() {
            if slot.device_type == DeviceType::Bjt && slot_idx < mna.nonlinear_devices.len() {
                let bp = if let DeviceParams::Bjt(bp) = &slot.params {
                    bp
                } else {
                    continue;
                };
                let dev = &mna.nonlinear_devices[slot_idx];
                let nc = dev.node_indices[0];
                let nb = dev.node_indices[1];
                let v_c = if nc > 0 && nc - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nc - 1]
                } else {
                    0.0
                };
                let v_b = if nb > 0 && nb - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nb - 1]
                } else {
                    0.0
                };
                let vbc = v_b - v_c;
                let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                let vbc_eff = sign * vbc;
                // FA reduction is only exact for pure Ebers-Moll devices:
                // the 1D emission has no qb (drops Early/IKF) and uses
                // Ib = Ic/BF (drops ISE leakage). Gummel-Poon or ISE-carded
                // BJTs must route full-2D — slower but correct. A frozen-qb
                // 1D enhancement is a recorded follow-up.
                //
                // Self-heating (finite RTH) is also excluded: the thermal
                // update reads the 2D slot pair (Ic at s, Ib at s+1 and the
                // matching Vbe/Vbc rows). A 1D FA slot would make s+1 alias
                // the NEXT device's slot (or run out of bounds).
                //
                // Ohmic parasitics (RB/RC/RE) are also excluded: the 1D FA
                // emission drops them entirely, and gm·RE reaches O(1) on
                // power BJTs (RE=0.5Ω @ 100 mA → gm·RE ≈ 1.9 — a several-x
                // transconductance error on exactly the devices FA targets).
                //
                // Threshold -0.5V provides adequate margin for audio-level signals
                // (typical stage Vce margin is 0.85V in cascaded topologies).
                if vbc_eff < -0.5 {
                    let name = dev.name.to_ascii_uppercase();

                    // `--bjt-fa off`: never reduce — every BJT stays full-2D.
                    if config.bjt_fa_mode == BjtFaMode::Off {
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V) but --bjt-fa=off — routing full-2D.",
                            name,
                            vbc_eff
                        );
                        continue;
                    }

                    // Self-heating is a STRUCTURAL exclusion, not an accuracy
                    // one: the thermal update reads the 2D (Ic,Ib) slot pair at
                    // (s, s+1); a 1D slot would alias the NEXT device's slot (or
                    // run out of bounds). It is NEVER reduced — not even under
                    // `--bjt-fa=force`.
                    if bp.has_self_heating() {
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V) but self-heating (RTH finite) — the thermal update needs the 2D (Ic,Ib) slot pair; NOT 1D-reduced even under --bjt-fa=force. Routing full-2D.",
                            name,
                            vbc_eff
                        );
                        continue;
                    }

                    // GP / ISE / ohmic-parasitic devices are ACCURACY-excluded:
                    // the 1D FA emission is structurally valid (uses IS/NF/Vt,
                    // Ib=Ic/BF) but drops qb / leakage / RB-RC-RE. Exact only for
                    // pure Ebers-Moll.
                    if bp.is_gummel_poon() || bp.has_ise() || bp.has_parasitics() {
                        let mechanism = if bp.is_gummel_poon() {
                            "Gummel-Poon params present (VAF/VAR/IKF/IKR) — 1D FA emission has no qb (drops Early effect + high-level injection)"
                        } else if bp.has_ise() {
                            "ISE leakage present — 1D FA emission uses Ib=Ic/BF"
                        } else {
                            "ohmic parasitics present (RB/RC/RE) — 1D FA emission drops them; gm·RE reaches O(1) on power BJTs"
                        };
                        if config.bjt_fa_mode == BjtFaMode::Force {
                            // Explicit user opt-in. The reduction is accuracy-
                            // lossy and NOT safe under signal: the collector
                            // swing modulates qb, which this compile-time
                            // decision cannot see. Warn loudly, per device.
                            crate::diag_warn!(
                                "BJT '{}' FORCE-reduced to 1D by --bjt-fa=force despite: {}. Accuracy is NOT guaranteed under signal (~1-2 dB deviation under hard drive for GP/ISE, larger for parasitics). You requested this — remove --bjt-fa=force for the accuracy-exact full-2D model.",
                                name,
                                mechanism
                            );
                            forward_active.insert(name);
                        } else {
                            // Auto (default): leave full-2D — exact.
                            log::info!(
                                "BJT '{}' is forward-active (Vbc={:.3}V) but NOT 1D-reduced: {}. Routing full-2D (use --bjt-fa=force to override).",
                                name,
                                vbc_eff,
                                mechanism
                            );
                            continue;
                        }
                    } else {
                        // Pure Ebers-Moll: 1D reduction is EXACT (auto + force).
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V). Using 1D model (exact).",
                            name,
                            vbc_eff
                        );
                        forward_active.insert(name);
                    }
                }
            }
        }
        forward_active
    }

    /// Per-device MNA node indices, parallel to `slots`.
    ///
    /// `build_device_info_with_mna` creates exactly one slot per entry of
    /// `mna.nonlinear_devices`, in the same order (linearized devices are
    /// absent from both). That 1:1 correspondence is what the FA / grid-off /
    /// LDR arms of that builder already rely on; this helper makes it
    /// explicit for the emitters. Returns an empty list (and warns) if the
    /// two ever disagree, so the region-exit characterization is simply
    /// not emitted rather than emitted against the wrong terminals.
    pub(super) fn device_node_indices_for(
        slots: &[DeviceSlot],
        mna: &crate::mna::MnaSystem,
    ) -> Vec<Vec<usize>> {
        if slots.len() != mna.nonlinear_devices.len() {
            crate::diag_warn!(
                "device_slots ({}) and mna.nonlinear_devices ({}) differ in length; \
                 diag_region_exit_count will not be emitted for this circuit",
                slots.len(),
                mna.nonlinear_devices.len()
            );
            return Vec::new();
        }
        mna.nonlinear_devices
            .iter()
            .map(|d| d.node_indices.clone())
            .collect()
    }

    /// Phase 1b grid-off pentode reduction — selection.
    ///
    /// Returns a `HashMap<String, f64>` mapping pentode name (uppercased)
    /// to the DC-OP-converged `Vg2k = V[screen] - V[cathode]` that the
    /// reduced 2D device freezes. The caller passes the map to
    /// [`MnaSystem::from_netlist_with_grid_off`] (rebuilds with
    /// `dimension: 2` pentode slots); [`build_device_info_with_mna`](Self::build_device_info_with_mna) then
    /// sets `TubeParams.kind = SharpPentodeGridOff` and `vg2k_frozen` from
    /// the reduced MNA.
    ///
    /// **`force_all == false` (`--tube-grid-fa auto`) never reduces.** The
    /// reduction drops two things that the full 3D model carries:
    ///
    /// 1. `Vg2k` as a live NR dimension. It is frozen at its DC value, but
    ///    `Vg2k = V[screen] - V[cathode]` is cathode-referenced: every
    ///    cathode-biased stage without a bypass capacitor, and every stage
    ///    with a finite screen impedance, has a signal-dependent `Vg2k`, and
    ///    the local negative feedback through `dIp/dVg2k` is lost. Measured
    ///    against ngspice as a small-signal gain error of +2.2% (EF86,
    ///    Rk 4.7k unbypassed), +3.0% (EL84, Rk 150 unbypassed, screen
    ///    bypassed) and +12.3% (EL84, Rk 130 and a 1k screen stop, both
    ///    unbypassed); the linearized prediction from the DC-OP
    ///    sensitivities reproduces all three to four digits. See
    ///    `DEVICE_MODELS.md` "Grid-Off Reduction".
    /// 2. `Ig1`. Exact only while `Vgk <= 0`; the region is classified once
    ///    from the DC OP and never re-checked, so a stage driven into grid
    ///    conduction silently runs a model with no grid current.
    ///
    /// Neither can be bounded at compile time from a quiescent bias point,
    /// so there is no sound automatic selection; `auto` is reserved for a
    /// reduction that is provably neutral (none exists yet) and keeps the
    /// full 3D model. `force_all == true` (`--tube-grid-fa on`) reduces
    /// every non-variable-mu pentode and warns per device.
    ///
    /// Mirrors [`detect_forward_active_bjts`](Self::detect_forward_active_bjts) for the BJT case.
    pub fn detect_grid_off_pentodes(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        force_all: bool,
    ) -> std::collections::HashMap<String, f64> {
        use crate::dc_op;

        // Must use build_device_info_with_mna so that FA-reduced BJTs (dim=1) produce
        // the correct start_idx values matching mna.m. Using build_device_info(netlist)
        // without MNA gives unreduced dimensions, causing v_nl OOB when FA reduction
        // has already happened (e.g. Q1 reduced 2D→1D shifts all subsequent start_idx).
        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna)).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashMap::new();
        }

        // Candidate pentodes: full-3D, sharp (non-variable-mu), with the
        // four terminals the reduction needs. Variable-mu pentodes (6K7,
        // EF89) are excluded outright — they exist for continuously varying
        // bias under sidechain control, the opposite of a frozen screen.
        // Schema `validate()` also rejects that combination.
        let candidates: Vec<(usize, &crate::device_types::TubeParams)> = device_slots
            .iter()
            .enumerate()
            .filter_map(|(slot_idx, slot)| {
                if slot.device_type != DeviceType::Tube || slot.dimension != 3 {
                    return None;
                }
                let tp = match &slot.params {
                    DeviceParams::Tube(tp) if tp.is_pentode() && !tp.is_variable_mu() => tp,
                    _ => return None,
                };
                let dev = mna.nonlinear_devices.get(slot_idx)?;
                // Pentode node order (from `categorize_element` in mna.rs):
                // [plate, grid, cathode, screen] with optional [, suppressor].
                if dev.node_indices.len() < 4 {
                    return None;
                }
                Some((slot_idx, tp))
            })
            .collect();
        if candidates.is_empty() {
            return std::collections::HashMap::new();
        }

        if !force_all {
            for (slot_idx, _) in &candidates {
                log::info!(
                    "Pentode '{}' keeps the full 3D model under --tube-grid-fa auto: the \
                     frozen-Vg2k reduction is not accuracy-neutral (it drops the \
                     Vg2k = V(screen) - V(cathode) feedback through the cathode and \
                     screen impedances, and the Ig1 grid current for Vgk > 0). \
                     `--tube-grid-fa on` opts in.",
                    mna.nonlinear_devices[*slot_idx].name.to_ascii_uppercase()
                );
            }
            return std::collections::HashMap::new();
        }

        let dc_result =
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config(mna, config));
        let v_at = |n: usize| -> f64 {
            if n > 0 && n - 1 < dc_result.v_node.len() {
                dc_result.v_node[n - 1]
            } else {
                0.0
            }
        };

        let mut grid_off = std::collections::HashMap::new();
        for (slot_idx, tp) in candidates {
            let dev = &mna.nonlinear_devices[slot_idx];
            let n_plate = dev.node_indices[0];
            let n_grid = dev.node_indices[1];
            let n_cathode = dev.node_indices[2];
            let n_screen = dev.node_indices[3];
            let v_cathode = v_at(n_cathode);
            let vgk = v_at(n_grid) - v_cathode;
            let vg2k = v_at(n_screen) - v_cathode;
            let vpk = v_at(n_plate) - v_cathode;
            let name = dev.name.to_ascii_uppercase();
            // Explicit user opt-in (`--tube-grid-fa on`). The reduction is
            // accuracy-lossy and NOT safe under signal: warn loudly, per
            // device, naming what is dropped. Mirrors `--bjt-fa force`.
            crate::diag_warn!(
                "Pentode '{}' FORCE-reduced to 2D by --tube-grid-fa on (Vgk={:.3}V, \
                 Vg2k={:.3}V frozen, Vpk={:.3}V). Dropped: (1) the live Vg2k = \
                 V(screen) - V(cathode) dimension — its feedback through the cathode \
                 and screen impedances is lost (measured +2% to +12% small-signal gain \
                 error on cathode-biased stages; exact only with an AC-grounded cathode \
                 AND screen); (2) the Ig1 grid current — wrong whenever Vgk > 0 (grid \
                 conduction; diag_region_exit_count counts those samples). Remove \
                 --tube-grid-fa on for the full 3D model.",
                name,
                vgk,
                vg2k,
                vpk
            );
            if vgk >= -(tp.vgk_onset + 0.5) {
                crate::diag_warn!(
                    "Pentode '{}' is NOT biased below grid cutoff at the DC OP \
                     (Vgk={:.3}V, onset {:.2}V): the forced grid-off model drops Ig1 \
                     at a bias where the grid already conducts.",
                    name,
                    vgk,
                    tp.vgk_onset
                );
            }
            grid_off.insert(name, vg2k);
        }
        grid_off
    }
}
