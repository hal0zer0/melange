# Reliability & Circuit Modeling Accuracy Review — 2026-03-21

5 specialist agents reviewed reliability and accuracy. Findings deduplicated and prioritized.

## Critical — Bugs affecting correctness

### C1. Multi-output plugin with params generates uncompilable code (Plugin Dev)
- `plugin_template.rs:503-525` — parametrized multi-output loop references `self.circuit_states[ch]` (plural) but multi-output struct uses `self.circuit_state` (singular)
- Affects ALL multi-output circuits with default flags (ear_protection=true, level_params=true)
- **Fix**: Use correct struct field name for multi-output path

### C2. Runtime GP q2 still uses NF*VT instead of plain VT (Analog Expert)
- `crates/melange-devices/src/bjt.rs:327-342,435-438` — runtime `BjtGummelPoon::qb()` and Jacobian use `nf_vt`/`nr_vt` for q2
- Codegen template was fixed (C3 from previous swarm) but runtime was not
- Causes codegen-vs-runtime discrepancy for BJTs with NF != 1.0
- **Fix**: Change runtime q2 to use plain `vt` (match codegen template)

### C3. Junction capacitances (CJO/CJE/CJC/CGS/CGD/CCG/CGP/CCP) not validated (Robustness)
- `ir.rs:1703,1801-1802,1989-1990,2051-2052,2143-2145` — all read with `.unwrap_or(0.0)`, no negative/NaN/Inf check
- Negative capacitance makes A matrix ill-conditioned; Inf capacitance causes singular A
- Every other device param is validated; capacitances are the gap
- **Fix**: Add `validate_non_negative_finite()` for all junction cap params

### C4. JFET/MOSFET RD/RS not validated for negativity or finiteness (Robustness)
- `ir.rs:1992-1994` (JFET), `ir.rs:2054-2056` (MOSFET) — read with `.unwrap_or(0.0)`, no validation
- BJT RB/RC/RE and diode RS ARE validated; JFET/MOSFET RS/RD are not
- Negative RS makes Jacobian correction denominator approach zero → wrong results
- **Fix**: Add same validation as BJT parasitic R

### C5. Parasitic cap auto-insertion not working in production (DSP)
- `mna.rs:1221-1308` — `add_parasitic_caps()` only used in tests, not production
- Production uses `stamp_device_junction_caps()` which only stamps if model params are nonzero
- A model without junction caps AND no explicit circuit caps → A=G, trapezoidal degenerates to forward Euler, unstable
- CLAUDE.md claims auto-insertion but it doesn't happen
- **Fix**: Check C matrix after stamping; if all-zero and nonlinear devices present, auto-insert 10pF parasitics

## High — Issues that produce wrong results under normal use

### H1. NaN reset sets i_nl_prev to zero instead of DC_NL_I (Plugin Dev)
- `process_sample.rs.tera:164-165` — `state.i_nl_prev = [0.0; M]` after setting `state.v_prev = state.dc_operating_point`
- Creates inconsistent state pair → large recovery transient, possible repeated NaN cascade
- `reset()` correctly uses `DC_NL_I`; NaN path doesn't
- **Fix**: Use `state.i_nl_prev = DC_NL_I` (with `{% if has_dc_nl %}` guard)

### H2. NaN reset doesn't clear oversampling filter state (Plugin Dev)
- `process_sample.rs.tera:161-188` — `os_up_state`/`os_dn_state` not zeroed on NaN
- Corrupt filter state produces garbage for several samples after NaN recovery
- `reset()` correctly zeroes all oversampling state
- **Fix**: Add oversampling state zeroing to NaN reset path

### H3. GUMMEL_POON.md says base current NOT modified by GP, but code divides ib_fwd by qb (Analog Expert)
- `docs/aidocs/GUMMEL_POON.md:77-82` — documentation is wrong, code is correct (SPICE3f5 does divide by qb)
- Jacobian section also shows wrong EM formulas instead of quotient rule
- Would mislead anyone maintaining the GP model
- **Fix**: Update docs to match code

### H4. Tube RGI not validated for finiteness (Robustness)
- `ir.rs:2149` — checks `rgi < 0.0` but not `!rgi.is_finite()`
- `RGI=inf` would pass validation, inner NR fails silently
- **Fix**: Add `|| !rgi.is_finite()` to the check

### H5. Failed DC OP silently baked into codegen as zeros (DSP)
- `dc_op.rs:1532-1541` — returns `i_nl=[0.0; m]` with `converged=false`
- Codegen embeds this as `DC_NL_I = [0.0, ...]` — all devices start "off"
- Startup transient, possibly hundreds of samples of wrong output
- **Fix**: Log a prominent warning or refuse to generate code when DC OP fails

### H6. GP q1_denom guard threshold too tight (Robustness)
- `device_bjt.rs.tera:22-23` — guard is `q1_denom.abs() < 1e-30`
- With small VAF/VAR (e.g., 1e-10, valid per validation), normal bias voltages make q1_denom massively negative
- `q1 = 1/(-1e7)` → tiny negative → collector current flips sign
- **Fix**: Clamp `q1_denom` to be positive: `q1_denom.max(1e-6)` or reject VAF/VAR < 1.0

## Medium — Issues that matter for robustness

### M1. JFET/MOSFET parasitic R corrects Jacobian but not current (Analog Expert)
- `device_jfet.rs.tera:74-87`, `device_mosfet.rs.tera:68-81` — drain current computed at external voltages, not internal junction voltages
- BJT does full inner NR for parasitic R; JFET/MOSFET use closed-form Jacobian-only correction
- Bias error proportional to I*R voltage drop
- **Fix**: Add inner NR like BJT, or document the approximation and its limits

### M2. `unreachable!()` in codegen NR match arms (API, Robustness)
- `rust_emitter.rs:3130,3172,3229,3285,3336` — panics on mismatched DeviceType/DeviceParams
- CircuitIR is Serialize/Deserialize, so corrupted JSON can trigger this
- **Fix**: Return `CodegenError::InvalidDevice` instead

### M3. `invert_small_matrix` silently returns identity on singular matrix (DSP, Robustness)
- `mna.rs:1530-1587` — transformer inductance matrix singular → identity with log::warn
- Produces wrong Y = T/2 * I, silently wrong transformer behavior
- **Fix**: Return `Result` instead of silent fallback

### M4. CircuitSolver/LinearSolver don't validate node indices (API)
- `solver.rs:677-776,1538-1557` — NodalSolver validates, others don't
- Out-of-bounds output_node silently returns 0.0
- **Fix**: Add same validation as NodalSolver::new()

### M5. RELTOL inconsistency between DK and Nodal paths (DSP)
- DK NR: RELTOL=1e-3, VNTOL=1e-6 (hardcoded)
- Full Nodal NR: RELTOL=1e-6 (1000x tighter)
- Same circuit converges faster on DK but with looser accuracy
- **Fix**: Use consistent RELTOL across all solver paths

### M6. Damping produces inconsistent v_prev/i_nl_prev state pair (Plugin Dev)
- `process_sample.rs.tera:130-159` — damped i_nl is linear blend, not actual nonlinear current at damped v
- Can cause sustained sub-threshold oscillation ("buzz")
- **Fix**: Re-evaluate device currents at damped voltages, or document the limitation

### M7. Singular Jacobian fallback uses fixed 0.01A clamp (DSP)
- `rust_emitter.rs:2757` — gradient-descent fallback hard-clamped to ±0.01A
- Insufficient for power amp circuits with 100mA+ collector currents
- **Fix**: Scale clamp with operating point or use device-specific limits

### M8. No floating node detection (API)
- `mna.rs` — nodes connected to only one terminal cause singular A
- Error message is generic "matrix singular", doesn't identify the floating node
- **Fix**: After MNA construction, warn on nodes with only one connection

## Low — Polish items

### L1. NR predictor guess not limited — first iteration can see extreme voltages (DSP)
### L2. No condition number tracking for per-sample NR Jacobian (DSP)
### L3. `fast_ln` produces wrong results for zero/negative input (latent) (DSP, Robustness)
### L4. BJT BF/BR=0 via deserialization causes division by zero (Robustness)
### L5. BJT IKF/IKR very small values kill transistor gain silently (Robustness)
### L6. MOSFET VT=0 accepted without warning — always-on device (Robustness)
### L7. Pot with both nodes grounded has no effect, no warning (Robustness)
### L8. DC block coefficient uses first-order approximation (Plugin Dev)
### L9. Double-counting diag_nr_max_iter_count in BE fallback path (Plugin Dev)
### L10. set_sample_rate doesn't reset v_prev/i_nl_prev to DC OP (Plugin Dev)
### L11. SolverConfig.tolerance emitted but ignored by DK path (DSP)
### L12. No M consistency check between device_info and kernel.m (API)
### L13. CodegenConfig::validate() doesn't check output_nodes or oversampling_factor (API)

## Positive Findings (confirmed working well)

- Zero `unsafe` code in entire workspace
- Zero `unwrap()`/`panic!()` in core library paths (ir.rs, dc_op.rs, codegen public API)
- SPICE pnjlim/fetlim applied consistently in all 3 solver paths (DK, Schur, full Nodal)
- Equilibrated matrix inversion with iterative refinement — professional-grade numerical stability
- DC OP convergence chain (Direct NR → Source Stepping → Gmin Stepping) — robust
- Input sanitization (NaN/Inf check + ±100V clamp) in all codegen paths
- Parser hardening thorough: rejects negative/zero/NaN/Inf, self-connections, subcircuit cycles
- Real-time safety: no heap allocation in any per-sample path
- fast_exp accuracy <0.0004%, safe edge cases, proper clamping
