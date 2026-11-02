//! JFET (Junction Field-Effect Transistor) models.

use crate::{safeguards, NonlinearDevice, VT_ROOM};

/// JFET channel type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JfetChannel {
    N,
    P,
}

/// Parker–Skellern drain-current law (SPICE JFET level 2): the parameters
/// that shape it beyond Shichman–Hodges, with ngspice's defaults.
///
/// The law is ngspice's `PSids` (jfet2/psmodel.c) with its trap-dispersion
/// and thermal-reduction terms at their zero defaults, where they are exact
/// identities: below pinch-off the gate overdrive is a softplus of scale
/// `VST·(1 + MVST·Vds)`, the current is a dual power law (`P` in the
/// triode region, `Q` saturated) with smooth early saturation (`Z`, `XI`,
/// `MXI`, `VBI`), times `BETA·(1 + LAMBDA·Vds)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParkerSkellern {
    /// Transconductance parameter BETA \[A/V^Q\]. Replaces `Jfet::idss`.
    pub beta: f64,
    /// Subthreshold potential VST \[V\] (0 = hard pinch-off edge). The
    /// deep-cutoff current e-folds every `VST/Q` volts.
    pub vst: f64,
    /// Drain modulation of VST \[1/V\].
    pub mvst: f64,
    /// Triode-region power law.
    pub p: f64,
    /// Saturated-region power law.
    pub q: f64,
    /// Saturation knee curvature.
    pub z: f64,
    /// Saturation index.
    pub xi: f64,
    /// Gate modulation of the saturation index.
    pub mxi: f64,
    /// Gate junction built-in potential (SPICE `PB`) \[V\].
    pub vbi: f64,
}

impl ParkerSkellern {
    /// ngspice's level-2 defaults, at the given BETA.
    pub fn with_beta(beta: f64) -> Self {
        Self {
            beta,
            vst: 0.0,
            mvst: 0.0,
            p: 2.0,
            q: 2.0,
            z: 1.0,
            xi: 1000.0,
            mxi: 0.0,
            vbi: 1.0,
        }
    }

    /// Normal-mode (`vds >= 0`) channel current and its partials
    /// `(Id, dId/dVgs, dId/dVds)` for an N-channel device with pinch-off
    /// `vto` (< 0). A line-for-line transcription of ngspice's `PSids` drain
    /// block (jfet2/psmodel.c), so the variable names are ngspice's.
    ///
    /// The law, in four stages, each feeding the next:
    /// 1. **Subthreshold**: the gate overdrive `vgst = Vgs − VTO` becomes the
    ///    softplus `vgt = vst·ln(1 + e^(vgst/vst))`, which is `vgst` well above
    ///    pinch-off and `vst·e^(vgst/vst)` well below it (the exponential
    ///    tail). `vst = VST·(1 + MVST·Vds)`.
    /// 2. **Dual power law**: `vdp = Vds·D3·vgt^(P−Q)` rescales the drain
    ///    voltage so the triode conductance grows as `vgt^(P−1)` while the
    ///    saturated current grows as `vgt^Q`.
    /// 3. **Early saturation**: `vdt` is a smooth minimum of `vdp` and the
    ///    saturation voltage `vsat` (≤ `vgt`), knee sharpness set by `Z`.
    /// 4. **Q-law**: `Id = vgt^Q − (vgt − vdt)^Q`, times `BETA·(1 + λ·Vds)`.
    ///    At P = Q = 2 with no early saturation this is Shichman–Hodges,
    ///    `2·vgt·Vds − Vds²` below saturation and `vgt²` above.
    ///
    /// The partials are carried alongside as `gm = ∂Id/∂(stage input)` and
    /// `gds = ∂Id/∂(stage drain variable)`, chained back through each stage.
    pub fn normal_mode(&self, vgs: f64, vds: f64, vto: f64, lambda: f64) -> (f64, f64, f64) {
        // ngspice's numerical guards on the softplus, in units of `vst`:
        // below `vgst = FX·vst` the device is in "extreme cut-off" and the
        // current is set to exactly 0. There `vgt ≈ e^−10·vst ≈ 4.5e-5·vst`,
        // and the smooth minimum of stage 3 is a difference of two nearly
        // equal square roots (`rpt − a_rpt`) that would be all rounding error
        // if `vgt` were let shrink further; the current at the edge is
        // ~BETA·(4.5e-5·vst)^Q, far below any solver tolerance.
        const FX: f64 = -10.0;
        // Above `vgst = MX·vst`, e^(vgst/vst) > e^40 ≈ 2.4e17 exceeds f64's
        // 2^53 resolution of `1 + e^x`, so the softplus already equals its
        // asymptote `vgst` to rounding; with a small VST the exponent reaches
        // the hundreds and `exp` overflows past x ≈ 709. The softplus is
        // continued linearly instead.
        const MX: f64 = 40.0;
        // e^MX, ngspice's literal `EMX`.
        const EMX: f64 = 2.3538526683702e+17;

        // Per-device constants (ngspice computes these once in
        // PSinstanceinit; here per call, because VP is a runtime parameter).
        // `woo`: the gate swing from pinch-off to the junction's built-in
        // potential, i.e. from a closed to a fully open channel.
        let woo = self.vbi - vto;
        // Saturation potential scale: `vsat` departs from `vgt` (velocity
        // saturation) once `vgt` is a sizeable fraction of XI·woo.
        let xi_woo = self.xi * woo;
        // Knee factor of the smooth minimum: makes `dvdt/dvdp = 1` at Vds = 0,
        // so the triode region starts with exactly the power-law slope.
        let za = (1.0 + self.z).sqrt() / 2.0;
        // Scales stage 2 so the triode conductance at Vds = 0 is
        // P·vgt^(P−1)/woo^(P−Q), the slope of the power law vgt^P/woo^(P−Q);
        // that law meets the saturated vgt^Q at full opening (`vgt = woo`).
        let d3 = self.p / self.q / woo.powf(self.p - self.q);

        let vdst = vds;
        let vgst = vgs - vto;
        let vst = self.vst * (1.0 + self.mvst * vdst);
        let (mut idrain, mut gm, mut gds);
        if vgst > FX * vst {
            // Stage 1, subthreshold softplus. `subfac = 1 + e^(vgst/vst)`;
            // its reciprocal later gives the softplus slope.
            let arg = MX * vst;
            let (subfac, vgt) = if vgst > arg {
                // Numerically large: the softplus is its asymptote, joined at
                // `vgst = MX·vst` with the softplus's own slope EMX/(1+EMX).
                let subfac = EMX + 1.0;
                (subfac, (EMX / subfac) * (vgst - arg) + arg)
            } else {
                let subfac = 1.0 + (vgst / vst).exp();
                (subfac, vst * subfac.ln())
            };
            // Stage 2, dual power law: the effective drain potential `vdp`.
            let m_q = self.q;
            let pmq = self.p - m_q;
            let dvpd_dvdst = d3 * vgt.powf(pmq); // ∂vdp/∂Vds
            let vdp = vdst * dvpd_dvdst;
            // Stage 3, early saturation. `vsat` is `vgt` reduced by velocity
            // saturation (`vsat → vgt` as XI → ∞).
            let vsat_fac = vgt / (self.mxi * vgt + xi_woo);
            let vsat = vgt / (1.0 + vsat_fac);
            // Smooth minimum of `vdp` and `vsat`:
            //   vdt = √(aa² + c) − √((aa − vsat)² + c),  aa = za·vdp + vsat/2,
            //   c = Z·vsat²/4.
            // It is 0 at vdp = 0, rises with slope 1, and tends to `vsat` as
            // vdp → ∞; Z → 0 makes it the hard min(vdp, vsat).
            let aa = za * vdp + vsat / 2.0;
            let a_aa = aa - vsat;
            let arg = vsat * vsat * self.z / 4.0;
            let rpt = (aa * aa + arg).sqrt();
            let a_rpt = (a_aa * a_aa + arg).sqrt();
            let vdt = rpt - a_rpt;
            let dvdt_dvdp = za * (aa / rpt - a_aa / a_rpt);
            // ∂vdt/∂vgt through `vsat` at fixed `vdp`. `vdt` is homogeneous of
            // degree 1 in (vdp, vsat), so ∂vdt/∂vsat = (vdt − vdp·∂vdt/∂vdp)/vsat
            // (Euler), and ∂vsat/∂vgt = (1 + MXI·f²)/(1 + f)² with f = vsat_fac.
            let dvdt_dvgt = (vdt - vdp * dvdt_dvdp) * (1.0 + self.mxi * vsat_fac * vsat_fac)
                / (1.0 + vsat_fac)
                / vgt;
            // Stage 4, the Q-law Id = vgt^Q − (vgt − vdt)^Q, written as
            // vdt·x + vgt·(vgt^(Q−1) − x) with x = (vgt − vdt)^(Q−1) so that
            // x also gives the partials: ∂Id/∂vdt = Q·x and, at fixed vdt,
            // ∂Id/∂vgt = Q·(vgt^(Q−1) − x).
            gds = (vgt - vdt).powf(m_q - 1.0);
            gm = vgt.powf(m_q - 1.0) - gds;
            idrain = vdt * gds + vgt * gm;
            gds *= m_q; // ∂Id/∂vdt
            gm *= m_q; // ∂Id/∂vgt at fixed vdt
                       // Chain back: vdt depends on vgt (via vsat) and on vdp.
            gm += gds * dvdt_dvgt;
            gds *= dvdt_dvdp; // ∂Id/∂vdp
                              // vdp depends on vgt through vgt^(P−Q): ∂vdp/∂vgt = (P−Q)·vdp/vgt.
            gm += gds * pmq * vdp / vgt; // ∂Id/∂vgt, complete
            gds *= dvpd_dvdst; // ∂Id/∂Vds at fixed vgt
                               // Softplus slope ∂vgt/∂vgst = e^x/(1 + e^x) = 1 − 1/subfac (the
                               // logistic sigmoid of x = vgst/vst).
            let arg = 1.0 - 1.0 / subfac;
            if vst != 0.0 {
                // MVST makes vst depend on Vds: ∂vgt/∂vst at fixed vgst is
                // (vgt − vgst·sigmoid)/vst, and ∂vst/∂Vds = VST·MVST.
                gds += gm * self.vst * self.mvst * (vgt - vgst * arg) / vst;
            }
            gm *= arg; // ∂Id/∂Vgs = ∂Id/∂vgt · ∂vgt/∂vgst
        } else {
            // Extreme cut-off (see FX): exactly no current and no conductance.
            idrain = 0.0;
            gm = 0.0;
            gds = 0.0;
        }
        // Channel-length modulation and BETA: Id·BETA·(1 + λ·Vds), whose Vds
        // derivative adds BETA·λ·Id.
        let arg = self.beta * (1.0 + lambda * vdst);
        gm *= arg;
        gds = self.beta * lambda * idrain + gds * arg;
        idrain *= arg;
        (idrain, gm, gds)
    }
}

/// Symmetric subthreshold channel law (melange JFET level 3): Shichman–Hodges
/// with a thermal tail below pinch-off, source/drain symmetric.
///
/// ```text
/// Id = BETA·(1 + LAMBDA·|Vds|)·[G(Vgs − VTO) − G(Vgd − VTO)]
/// G(x) = (2·n·Vt · ln(1 + e^(x/(2·n·Vt))))²      (a squared softplus)
/// ```
///
/// `G(x)` is `x²` for `x ≫ 2nVt` and `(2nVt)²·e^(x/(nVt))` for `x ≪ −2nVt`,
/// so above pinch-off the law is Shichman–Hodges (triode `2·Vgst·Vds − Vds²`
/// where `Vgd > VTO`, saturation `Vgst²` where `Vgd ≤ VTO`, `BETA =
/// IDSS/VTO²`) and below it the channel conducts by a gate-controlled barrier
/// current that e-folds every `n·Vt` of gate voltage, forward minus reverse:
/// a pinched channel stays a resistor over a few `Vt` of drain swing at any
/// depth, instead of switching off at a hard edge. As `Vt → 0`, `G(x) →
/// max(x, 0)²` and the law is Shichman–Hodges exactly, for every `n`. The
/// form is EKV's forward-minus-reverse interpolation (Enz, Krummenacher,
/// Vittoz 1995, DOI 10.1007/BF01239381) applied to the JFET by analogy: in a
/// channel between tied gates the barrier is referenced equally to both
/// ends, so the current is `h(Vgs) − h(Vgd)` and the even-order null of a
/// half-drain-fed gate stays at one half for any `n`, as in the classical
/// JFET laws. No held JFET source; see DEVICE_MODELS.md.
///
/// The law is odd under the source/drain swap `(Vgs, Vds) → (Vgd, −Vds)`, so
/// it needs no SPICE mode branch and is evaluated as written at both signs
/// of `Vds`. There is no cut-off edge or branch: the current below pinch-off
/// is small and continuous, and underflows to 0 only where `e^(x/(2nVt))`
/// squared does, some 18 V below pinch-off at `n = 1`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SymmetricSubthreshold {
    /// Subthreshold slope factor `n` (card `NSLOPE`, ≥ 1; 1 is the ideal
    /// long-channel slope of one e-fold per `Vt`). This IS `n`; it is not
    /// BSIM's `NFACTOR`, which is one coefficient inside `n`.
    pub n: f64,
    /// Thermal voltage `Vt = kT/q` \[V\] at the device temperature. The JFET
    /// has no temperature law, so this is [`VT_ROOM`] (300.15 K) as for its
    /// gate junctions; a field so the `Vt → 0` limit is testable.
    pub vt: f64,
}

impl Default for SymmetricSubthreshold {
    /// Ideal slope (`n = 1`) at [`VT_ROOM`].
    fn default() -> Self {
        Self {
            n: 1.0,
            vt: VT_ROOM,
        }
    }
}

impl SymmetricSubthreshold {
    /// The law's voltage scale `2·n·Vt` \[V\]: the softplus argument scale,
    /// which is also the width of the knee at pinch-off.
    pub fn scale(&self) -> f64 {
        2.0 * self.n * self.vt
    }

    /// Channel current and its partials `(Id, dId/dVgs, dId/dVds)` for an
    /// N-channel device with pinch-off `vto` (< 0), at the strength
    /// `beta = IDSS/VTO²` and channel-length modulation `lambda`.
    ///
    /// Steps:
    /// 1. The two gate overdrives, source side `u_s = Vgs − VTO` and drain
    ///    side `u_d = Vgd − VTO = u_s − Vds`.
    /// 2. Each through the squared softplus `G` and its derivative `G'(x) =
    ///    2·sp(x)·σ(x/a)`, `a = 2nVt`, `sp(x) = a·ln(1 + e^(x/a))` and `σ`
    ///    the logistic sigmoid (the softplus's slope). The softplus is
    ///    evaluated as `a·(max(z, 0) + ln(1 + e^(−|z|)))`, `z = x/a`, so no
    ///    exponent overflows: far above pinch-off it is `x` exactly, far
    ///    below it underflows smoothly toward 0.
    /// 3. `core = G(u_s) − G(u_d)`, which is odd under the source/drain swap
    ///    (IEEE subtraction is exactly antisymmetric) and exactly 0 at
    ///    `Vds = 0`. Its partials: `∂core/∂Vgs = G'(u_s) − G'(u_d)` (both
    ///    overdrives move with the gate) and `∂core/∂Vds = G'(u_d)` (only
    ///    `u_d` moves, by `−Vds`).
    /// 4. `Id = beta·core·(1 + lambda·|Vds|)`, whose `Vds` derivative adds
    ///    `beta·core·lambda·sign(Vds)`; `core` is 0 at `Vds = 0`, so `gds`
    ///    is continuous there.
    pub fn channel(&self, vgs: f64, vds: f64, vto: f64, beta: f64, lambda: f64) -> (f64, f64, f64) {
        let a = self.scale();
        // Squared softplus G(x) and its derivative G'(x), see steps 2.
        let g = |x: f64| -> (f64, f64) {
            let z = x / a;
            let sp = a * (z.max(0.0) + (-z.abs()).exp().ln_1p());
            let sigma = 1.0 / (1.0 + (-z).exp());
            (sp * sp, 2.0 * sp * sigma)
        };
        let u_s = vgs - vto;
        let u_d = u_s - vds;
        let (g_s, dg_s) = g(u_s);
        let (g_d, dg_d) = g(u_d);
        let core = g_s - g_d;
        let clm = 1.0 + lambda * vds.abs();
        let id = beta * core * clm;
        let gm = beta * (dg_s - dg_d) * clm;
        let gds = beta * (dg_d * clm + core * lambda * vds.signum());
        (id, gm, gds)
    }
}

/// JFET model: Shichman–Hodges (SPICE level 1), Parker–Skellern (SPICE
/// level 2) when `ps` is set, or the symmetric subthreshold law (melange
/// level 3) when `sym` is set.
///
/// For N-channel: negative Vgs to control, positive Vds
/// For P-channel: positive Vgs to control, negative Vds
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Jfet {
    /// Channel type
    pub channel: JfetChannel,
    /// Pinch-off voltage \[V\] (always negative for N-channel)
    pub vp: f64,
    /// Saturation current \[A\] (IDSS)
    pub idss: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// Gate junction saturation current \[A\] (SPICE `IS`, default 1e-14);
    /// 0 disables both gate junctions.
    pub is: f64,
    /// Gate junction emission coefficient (SPICE `N`, default 1).
    pub n: f64,
    /// Parker–Skellern channel law (level 2). `None` is Shichman–Hodges,
    /// from `idss`; `Some` replaces it, and `idss` is unused.
    pub ps: Option<ParkerSkellern>,
    /// Symmetric subthreshold channel law (level 3), from `idss` and `vp`
    /// like Shichman–Hodges. Exclusive with `ps`.
    pub sym: Option<SymmetricSubthreshold>,
}

impl Jfet {
    /// Create a new JFET.
    ///
    /// # Panics
    /// Panics if `vp` is zero (would cause division by zero) or `idss` is not positive.
    pub fn new(channel: JfetChannel, vp: f64, idss: f64) -> Self {
        assert!(vp.abs() > 1e-15, "JFET Vp must be non-zero, got {}", vp);
        assert!(idss > 0.0, "JFET IDSS must be positive, got {}", idss);
        Self {
            channel,
            vp,
            idss,
            lambda: 0.001,
            is: 1e-14,
            n: 1.0,
            ps: None,
            sym: None,
        }
    }

    /// A Parker–Skellern (level 2) JFET with pinch-off `vto` in melange's
    /// convention (negative N-channel, positive P-channel), as [`Self::new`].
    /// `idss` is set to `BETA·VTO²` for display only; the law reads `ps`.
    pub fn parker_skellern(channel: JfetChannel, vto: f64, ps: ParkerSkellern) -> Self {
        assert!(ps.beta > 0.0, "JFET BETA must be positive, got {}", ps.beta);
        let mut j = Self::new(channel, vto, ps.beta * vto * vto);
        j.ps = Some(ps);
        j
    }

    /// A symmetric-subthreshold (level 3) JFET: Shichman–Hodges strength
    /// `idss` at pinch-off `vp` (melange's convention, as [`Self::new`]),
    /// with `sym`'s thermal tail below pinch-off.
    pub fn symmetric_subthreshold(
        channel: JfetChannel,
        vp: f64,
        idss: f64,
        sym: SymmetricSubthreshold,
    ) -> Self {
        assert!(
            sym.n >= 1.0 && sym.n.is_finite(),
            "JFET NSLOPE must be at least 1, got {}",
            sym.n
        );
        assert!(sym.vt > 0.0, "JFET Vt must be positive, got {}", sym.vt);
        let mut j = Self::new(channel, vp, idss);
        j.sym = Some(sym);
        j
    }

    /// 2N5457 N-channel JFET.
    pub fn n_2n5457() -> Self {
        let c = crate::catalog::jfets::lookup("2N5457").expect("2N5457 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// J201 N-channel JFET (common in audio).
    pub fn n_j201() -> Self {
        let c = crate::catalog::jfets::lookup("J201").expect("J201 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// 2N3819 N-channel JFET.
    pub fn n_2n3819() -> Self {
        let c = crate::catalog::jfets::lookup("2N3819").expect("2N3819 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// 2N5460 P-channel JFET.
    pub fn p_2n5460() -> Self {
        let c = crate::catalog::jfets::lookup("2N5460").expect("2N5460 catalog entry");
        let mut j = Self::new(JfetChannel::P, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    fn sign(&self) -> f64 {
        match self.channel {
            JfetChannel::N => 1.0,
            JfetChannel::P => -1.0,
        }
    }

    /// Drain current given Vgs and Vds.
    ///
    /// SPICE-style mode handling: for `vds_eff >= 0` (normal mode) the
    /// source-referenced overdrive `Vgs - Vp` governs. For `vds_eff < 0`
    /// (reverse mode) the drain acts as the source, so the drain-referenced
    /// overdrive `Vgd - Vp` governs: the normal-mode equations are evaluated
    /// at `(vgd, -vds)` and the result is negated. The triode expression is
    /// exactly symmetric under this swap, so value and all Jacobian entries
    /// are continuous across Vds = 0.
    pub fn drain_current(&self, vgs: f64, vds: f64) -> f64 {
        let s = self.sign();

        // Convert to N-channel equivalent voltages
        // After sign flip, both N and P channel use N-channel equations
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        // For P-channel JFET:
        // - Constructor stores vp as positive (e.g., 2.5V)
        // - To use N-channel equations, we need vp_eff to be negative (-2.5V)
        // - vgst = vgs_eff - vp_eff = -Vgs - (-Vp) = Vp - Vgs
        // - For conduction: vgst > 0 means Vp > Vgs (correct for P-channel!)
        let vp_eff = match self.channel {
            JfetChannel::N => self.vp,  // Already negative
            JfetChannel::P => -self.vp, // Flip to negative
        };

        // The symmetric law is odd under the source/drain swap: evaluated as
        // written at either sign of Vds, no mode branch.
        if let Some(sym) = &self.sym {
            let beta = self.idss / (self.vp * self.vp);
            return s * sym.channel(vgs_eff, vds_eff, vp_eff, beta, self.lambda).0;
        }

        // Mode selection: vc = controlling gate overdrive reference,
        // vd = |vds_eff| >= 0, m = polarity of the drain current.
        let (vc, vd, m) = if vds_eff >= 0.0 {
            (vgs_eff, vds_eff, 1.0)
        } else {
            // Reverse mode: drain acts as source. vgd_eff = vgs_eff - vds_eff.
            (vgs_eff - vds_eff, -vds_eff, -1.0)
        };
        if let Some(ps) = &self.ps {
            return s * m * ps.normal_mode(vc, vd, vp_eff, self.lambda).0;
        }
        let vgst = vc - vp_eff;

        if vgst <= 0.0 {
            // Subthreshold: weak exponential for smooth NR convergence.
            // Gated by tanh(vd/2VT) so the leakage is continuous (and zero)
            // at Vds = 0 where the mode polarity m flips sign.
            let sub = 1e-12 * (vgst / (2.0 * VT_ROOM)).exp().min(1.0);
            return s * m * sub * (vd / (2.0 * VT_ROOM)).tanh();
        }

        // Saturation voltage (|Vds| at which device enters saturation)
        let vds_sat = vgst;

        let vp_abs = self.vp.abs();
        // Channel-length modulation uses the mode-consistent |Vds| (SPICE
        // behavior — see DEVICE_MODELS.md, `1 + lambda*|Vds|`).
        let clm = 1.0 + self.lambda * vd;

        if vd < vds_sat {
            // Linear (triode) region
            // Id = (2*IDSS/Vp^2) * ((Vgs-Vp)*Vds - Vds^2/2)
            let id = self.idss / (vp_abs * vp_abs) * (2.0 * vgst * vd - vd * vd);
            s * m * id * clm
        } else {
            // Saturation region
            // Id = IDSS * (1 - (Vgs-Vp)/Vp)^2 = IDSS * (Vgst/Vp)^2
            let id = self.idss * (vgst / vp_abs).powi(2);
            s * m * id * clm
        }
    }

    /// Partial derivatives for Jacobian.
    ///
    /// Returns (∂Id/∂Vgs, ∂Id/∂Vds)
    pub fn jacobian_partial(&self, vgs: f64, vds: f64) -> (f64, f64) {
        let s = self.sign();
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        let vp_eff = match self.channel {
            JfetChannel::N => self.vp,
            JfetChannel::P => -self.vp,
        };

        // Symmetric law: Id = s·f(s·vgs, s·vds), so with s² = 1 the partials
        // to the external voltages are f's own.
        if let Some(sym) = &self.sym {
            let beta = self.idss / (self.vp * self.vp);
            let (_, gm, gds) = sym.channel(vgs_eff, vds_eff, vp_eff, beta, self.lambda);
            return (gm, gds);
        }

        // Mode selection — must mirror drain_current exactly.
        // Id = s * m * f(vc, vd), with:
        //   normal  (m=+1): vc = vgs_eff,           vd = vds_eff
        //   reverse (m=-1): vc = vgs_eff - vds_eff, vd = -vds_eff
        // Chain rule to external (vgs, vds), with s² = 1:
        //   normal:  dId/dVgs = f_c            dId/dVds = f_d
        //   reverse: dId/dVgs = -f_c           dId/dVds = f_c + f_d
        let (vc, vd, m) = if vds_eff >= 0.0 {
            (vgs_eff, vds_eff, 1.0)
        } else {
            (vgs_eff - vds_eff, -vds_eff, -1.0)
        };
        let vgst = vc - vp_eff;

        // (f_c, f_d) = (∂f/∂vc, ∂f/∂vd)
        let (f_c, f_d) = if let Some(ps) = &self.ps {
            let (_, gm, gds) = ps.normal_mode(vc, vd, vp_eff, self.lambda);
            (gm, gds)
        } else if vgst <= 0.0 {
            // Subthreshold: derivative of tanh-gated weak exponential.
            // Matches device_jfet.rs.tera — unconditional, no dead branch.
            let sub = 1e-12 * (vgst / (2.0 * VT_ROOM)).exp().min(1.0);
            let t = (vd / (2.0 * VT_ROOM)).tanh();
            (
                sub * t / (2.0 * VT_ROOM),
                sub * (1.0 - t * t) / (2.0 * VT_ROOM),
            )
        } else {
            let vds_sat = vgst;
            let vp_abs = self.vp.abs();
            let vp2 = vp_abs * vp_abs;
            let clm = 1.0 + self.lambda * vd;

            if vd < vds_sat {
                // Linear (triode) region
                let f_c = self.idss * 2.0 * vd / vp2 * clm;
                let f_d_base = self.idss / vp2 * (2.0 * vgst - 2.0 * vd);
                let f_d_lambda = self.idss / vp2 * (2.0 * vgst * vd - vd * vd) * self.lambda;
                (f_c, f_d_base * clm + f_d_lambda)
            } else {
                // Saturation region
                let f_c = self.idss * 2.0 * vgst / vp2 * clm;
                let f_d = self.idss * (vgst / vp_abs).powi(2) * self.lambda;
                (f_c, f_d)
            }
        };

        if m >= 0.0 {
            (f_c, f_d)
        } else {
            (-f_c, f_c + f_d)
        }
    }

    /// The gate's pn junctions to the channel, source and drain side:
    /// `(Igs, Igd, g_gs, g_gd)`, each `IS·(exp(V/(N·Vt)) − 1)` at
    /// `Vgs` and `Vgd = Vgs − Vds` in the device's polarity, currents flowing
    /// into the gate. SPICE level-1 JFET gate diodes, without SPICE's GMIN
    /// conditioning term (the fixed point is the device's). The exponential is
    /// the IS-aware [`safeguards::junction_exp`].
    pub fn gate_junctions(&self, vgs: f64, vds: f64) -> (f64, f64, f64, f64) {
        if self.is == 0.0 {
            return (0.0, 0.0, 0.0, 0.0);
        }
        let s = self.sign();
        let n_vt = self.n * VT_ROOM;
        let junction = |v: f64| {
            let (e, de) = safeguards::junction_exp(s * v / n_vt, self.is);
            (s * self.is * (e - 1.0), self.is / n_vt * de)
        };
        let (igs, g_gs) = junction(vgs);
        let (igd, g_gd) = junction(vgs - vds);
        (igs, igd, g_gs, g_gd)
    }

    /// Terminal currents and Jacobian of the whole device at `(Vgs, Vds)`:
    /// `(I_drain, I_gate, [dId/dVgs, dId/dVds, dIg/dVgs, dIg/dVds])`, the
    /// channel plus both gate junctions. The drain loses the gate-drain
    /// junction's current, the gate carries both, the source the rest.
    pub fn evaluate(&self, vgs: f64, vds: f64) -> (f64, f64, [f64; 4]) {
        let id = self.drain_current(vgs, vds);
        let (gm, gds) = self.jacobian_partial(vgs, vds);
        let (igs, igd, g_gs, g_gd) = self.gate_junctions(vgs, vds);
        (
            id - igd,
            igs + igd,
            [gm - g_gd, gds + g_gd, g_gs + g_gd, -g_gd],
        )
    }
}

impl NonlinearDevice<2> for Jfet {
    /// Input: [Vgs, Vds]
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.drain_current(v[0], v[1])
    }

    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let (d_id_d_vgs, d_id_d_vds) = self.jacobian_partial(v[0], v[1]);
        [d_id_d_vgs, d_id_d_vds]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jfet_cutoff() {
        let jfet = Jfet::n_j201();

        // Vgs more negative than Vp: cutoff
        // Vp = -0.8, so Vgs = -2.0 < -0.8 should cutoff
        let id = jfet.drain_current(-2.0, 5.0);
        assert!(id.abs() < 1e-10);
    }

    #[test]
    fn test_jfet_saturation() {
        let jfet = Jfet::n_j201();

        // Vgs = 0, Vds > Vgs - Vp: saturation at IDSS
        // Vgst = 0 - (-0.8) = 0.8
        let id = jfet.drain_current(0.0, 5.0);

        // Should be close to IDSS (with channel length modulation)
        assert!(id > 0.0);
        assert!(id > jfet.idss * 0.5); // At least half of IDSS
    }

    #[test]
    fn test_jfet_idss() {
        let jfet = Jfet::n_j201();

        // At Vgs = 0, Id should be approximately IDSS
        let id = jfet.drain_current(0.0, 9.0);

        // Allow for channel length modulation
        assert!(id > jfet.idss); // lambda > 0 increases current at high Vds
    }

    #[test]
    fn test_jfet_polarity() {
        let n_jfet = Jfet::n_j201();
        let p_jfet = Jfet::p_2n5460();

        // N-channel: positive Vds, small negative Vgs (above Vp to conduct)
        // Vp = -0.8, Vgs = -0.3 > -0.8, so conducts
        let id_n = n_jfet.drain_current(-0.3, 5.0);

        // P-channel: negative Vds, small positive Vgs (below Vp to conduct)
        // Vp = 2.5, Vgs = 2.0 < 2.5, so conducts
        let id_p = p_jfet.drain_current(2.0, -5.0);

        assert!(
            id_n > 0.0,
            "N-channel current should be positive, got {}",
            id_n
        );
        assert!(
            id_p < 0.0,
            "P-channel current should be negative, got {}",
            id_p
        );
    }

    #[test]
    fn test_jfet_jacobian() {
        let jfet = Jfet::n_j201();

        let vgs = -0.3;
        let vds = 5.0;

        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Transconductance should be positive
        assert!(
            d_id_d_vgs > 0.0,
            "gm should be positive, got {}",
            d_id_d_vgs
        );

        // Output conductance should be positive
        assert!(
            d_id_d_vds > 0.0,
            "gds should be positive, got {}",
            d_id_d_vds
        );
    }

    #[test]
    fn test_jfet_jacobian_numerical() {
        let jfet = Jfet::n_j201();

        let vgs = -0.3;
        let vds = 5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Numerical verification
        let dv = 1e-6;
        let id = jfet.drain_current(vgs, vds);
        let id_vgs = jfet.drain_current(vgs + dv, vds);
        let id_vds = jfet.drain_current(vgs, vds + dv);

        let num_d_id_d_vgs = (id_vgs - id) / dv;
        let num_d_id_d_vds = (id_vds - id) / dv;

        // Should match within 1%
        assert!((d_id_d_vgs - num_d_id_d_vgs).abs() / d_id_d_vgs.abs() < 0.01);
        assert!((d_id_d_vds - num_d_id_d_vds).abs() / d_id_d_vds.abs() < 0.01);
    }

    #[test]
    fn test_jfet_p_channel_jacobian_numerical() {
        let jfet = Jfet::p_2n5460();

        // P-channel: positive Vgs (below Vp to conduct), negative Vds
        let vgs = 2.0;
        let vds = -5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Numerical verification (central difference)
        let eps = 1e-6;
        let num_d_id_d_vgs =
            (jfet.drain_current(vgs + eps, vds) - jfet.drain_current(vgs - eps, vds)) / (2.0 * eps);
        let num_d_id_d_vds =
            (jfet.drain_current(vgs, vds + eps) - jfet.drain_current(vgs, vds - eps)) / (2.0 * eps);

        // Should match within 1%
        assert!(
            (d_id_d_vgs - num_d_id_d_vgs).abs() / num_d_id_d_vgs.abs().max(1e-15) < 0.01,
            "P-channel gm: analytical={:.6e}, numerical={:.6e}",
            d_id_d_vgs,
            num_d_id_d_vgs
        );
        assert!(
            (d_id_d_vds - num_d_id_d_vds).abs() / num_d_id_d_vds.abs().max(1e-15) < 0.01,
            "P-channel gds: analytical={:.6e}, numerical={:.6e}",
            d_id_d_vds,
            num_d_id_d_vds
        );
    }

    /// Comprehensive multi-region FD Jacobian test covering cutoff boundary,
    /// linear/triode, saturation, and P-channel operation.
    #[test]
    fn test_jfet_multi_region_fd_jacobian() {
        let eps = 1e-7;

        // N-channel: Vp = -0.8, IDSS = 0.3e-3
        let n_jfet = Jfet::n_j201();

        // Operating points covering all regions:
        // (Vgs, Vds, description)
        let n_points: &[(f64, f64, &str)] = &[
            // Cutoff boundary: Vgs just above Vp
            (-0.75, 5.0, "N cutoff boundary"),
            // Saturation: Vgs=0, Vds >> Vgs-Vp
            (0.0, 5.0, "N saturation Vgs=0"),
            (-0.3, 5.0, "N saturation Vgs=-0.3"),
            (-0.5, 3.0, "N saturation Vgs=-0.5"),
            // Linear/triode: Vds < Vgs - Vp
            (0.0, 0.2, "N linear Vds=0.2"),
            (0.0, 0.5, "N linear Vds=0.5"),
            (-0.3, 0.1, "N linear Vgs=-0.3 Vds=0.1"),
        ];

        for &(vgs, vds, desc) in n_points {
            let (d_id_d_vgs, d_id_d_vds) = n_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (n_jfet.drain_current(vgs + eps, vds)
                - n_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (n_jfet.drain_current(vgs, vds + eps)
                - n_jfet.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "N-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }

        // P-channel: Vp = 2.5, IDSS = 5e-3
        let p_jfet = Jfet::p_2n5460();

        let p_points: &[(f64, f64, &str)] = &[
            // P-channel cutoff boundary: Vgs just below Vp
            (2.4, -5.0, "P cutoff boundary"),
            // P-channel saturation: Vgs=0, Vds negative
            (0.0, -5.0, "P saturation Vgs=0"),
            (1.0, -5.0, "P saturation Vgs=1.0"),
            // P-channel linear: |Vds| < |Vgs - Vp|
            (0.0, -0.5, "P linear Vds=-0.5"),
            (1.0, -0.2, "P linear Vgs=1 Vds=-0.2"),
        ];

        for &(vgs, vds, desc) in p_points {
            let (d_id_d_vgs, d_id_d_vds) = p_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (p_jfet.drain_current(vgs + eps, vds)
                - p_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (p_jfet.drain_current(vgs, vds + eps)
                - p_jfet.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "P-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "Vp must be non-zero")]
    fn test_jfet_vp_zero_rejected() {
        let _ = Jfet::new(JfetChannel::N, 0.0, 1e-3);
    }

    #[test]
    #[should_panic(expected = "IDSS must be positive")]
    fn test_jfet_negative_idss_rejected() {
        let _ = Jfet::new(JfetChannel::N, -2.0, -1e-3);
    }

    #[test]
    fn test_jfet_subthreshold_gm_boundary() {
        // Regression: prior code had `if sub < 1e-12 { ... } else { 0.0 }`
        // which incorrectly returned gm=0 at the exact vgst=0 boundary
        // where sub = 1e-12 exactly. Codegen template has the unconditional
        // form; runtime must match.
        let jfet = Jfet::new(JfetChannel::N, -2.5, 10e-3);
        let (gm, gds) = jfet.jacobian_partial(-2.5, 1.0); // vgs == vp -> vgst = 0
                                                          // tanh(vd/2VT) gate at vd=1.0 is 1.0 to machine precision.
        let expected_gm = 1e-12 / (2.0 * crate::VT_ROOM);
        assert!(
            (gm - expected_gm).abs() / expected_gm < 1e-9,
            "subthreshold gm at vgst=0: got {}, expected {}",
            gm,
            expected_gm
        );
        // gds carries the tanh-gate derivative sub*(1-t^2)/(2VT); at vd=1.0
        // sech^2 has fully decayed, so it is zero to well below any solver
        // tolerance (< 1e-24 A/V).
        assert!(
            gds.abs() < 1e-24,
            "subthreshold gds at vgst=0, vds=1: got {}",
            gds
        );
    }

    /// Reverse-saturation quadrant: J201 (Vp=-0.8) at Vgs=-1.0, Vds=-0.5.
    /// Vgs is below pinch-off but Vgd = -0.5 > Vp, so the drain acts as
    /// source and the device conducts reverse saturation:
    ///   Id = -IDSS*((Vgd-Vp)/|Vp|)^2 * (1+lambda*|Vds|)
    ///      = -6e-4*(0.3/0.8)^2*(1.002) ≈ -84.5 µA
    /// The old source-referenced-only cutoff returned ~1e-12 A here with
    /// dId/dVds = 0 (dead negative half-cycle in variable-resistor circuits).
    #[test]
    fn test_jfet_reverse_saturation_conducts() {
        let jfet = Jfet::n_j201();

        let id = jfet.drain_current(-1.0, -0.5);
        let expected = -jfet.idss * (0.3f64 / 0.8).powi(2) * (1.0 + jfet.lambda * 0.5);
        assert!(
            (id - expected).abs() / expected.abs() < 1e-12,
            "J201 reverse saturation: got {:.6e}, expected {:.6e}",
            id,
            expected
        );
        assert!(
            id < -80e-6 && id > -90e-6,
            "J201 at Vgs=-1.0, Vds=-0.5 should conduct ≈ -85 µA, got {:.3} µA",
            id * 1e6
        );

        // dId/dVds must be a real conductance (not zero) in this regime.
        let (gm, gds) = jfet.jacobian_partial(-1.0, -0.5);
        assert!(
            gds > 1e-6,
            "reverse-mode gds should be substantial, got {:.3e}",
            gds
        );
        // Raising Vgs raises Vgd -> more reverse conduction -> Id more negative.
        assert!(
            gm < 0.0,
            "reverse-mode gm should be negative, got {:.3e}",
            gm
        );
    }

    /// FD Jacobian checks in the reverse quadrant (Vds < 0 for N-channel),
    /// covering reverse triode, reverse saturation, points straddling the
    /// Vds=0 mode boundary, and points straddling the reverse cutoff
    /// boundary. Matches the multi-region FD test style.
    #[test]
    fn test_jfet_reverse_mode_fd_jacobian() {
        let eps = 1e-7;

        // N-channel J201: Vp = -0.8
        let n_jfet = Jfet::n_j201();
        let n_points: &[(f64, f64, &str)] = &[
            // Reverse saturation (Vgs below Vp, Vgd above Vp)
            (-1.0, -0.5, "N reverse saturation (trigger point)"),
            (-1.0, -0.35, "N reverse saturation shallow"),
            // Reverse triode (both overdrives positive)
            (-0.3, -0.2, "N reverse triode"),
            (0.0, -0.3, "N reverse triode Vgs=0"),
            // Straddling the Vds=0 mode boundary while conducting
            (-0.3, 0.02, "N mode boundary + side"),
            (-0.3, -0.02, "N mode boundary - side"),
            (0.0, 0.01, "N mode boundary + side Vgs=0"),
            (0.0, -0.01, "N mode boundary - side Vgs=0"),
            // Straddling the reverse cutoff boundary (Vgd crosses Vp):
            // Vgd = Vgs - Vds = -0.75 / -0.85 around Vp = -0.8
            (-1.0, -0.25, "N reverse just conducting"),
            (-1.0, -0.15, "N reverse just cutoff"),
        ];

        for &(vgs, vds, desc) in n_points {
            let (d_id_d_vgs, d_id_d_vds) = n_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (n_jfet.drain_current(vgs + eps, vds)
                - n_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (n_jfet.drain_current(vgs, vds + eps)
                - n_jfet.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "N-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }

        // P-channel 2N5460: Vp = 2.5; reverse quadrant is Vds > 0.
        let p_jfet = Jfet::p_2n5460();
        let p_points: &[(f64, f64, &str)] = &[
            // Reverse saturation: Vgs beyond Vp, Vgd within
            (3.0, 1.0, "P reverse saturation"),
            // Reverse triode
            (1.0, 0.5, "P reverse triode"),
            // Straddling the Vds=0 mode boundary
            (1.0, 0.02, "P mode boundary + side"),
            (1.0, -0.02, "P mode boundary - side"),
        ];

        for &(vgs, vds, desc) in p_points {
            let (d_id_d_vgs, d_id_d_vds) = p_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (p_jfet.drain_current(vgs + eps, vds)
                - p_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (p_jfet.drain_current(vgs, vds + eps)
                - p_jfet.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "P-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    /// Value and Jacobian continuity across the Vds=0 mode boundary, in
    /// conduction, subthreshold, and at the pinch-off boundary itself.
    #[test]
    fn test_jfet_continuity_across_vds_zero() {
        let jfet = Jfet::n_j201();
        let eps = 1e-9;

        for &vgs in &[0.0, -0.3, -0.5, -0.8, -1.0, -2.0] {
            let id_pos = jfet.drain_current(vgs, eps);
            let id_neg = jfet.drain_current(vgs, -eps);
            // Continuous function: |Id(+eps) - Id(-eps)| is O(2*eps*gds),
            // not zero. gds here is at most ~1.5e-3 S -> bound 1e-8 A.
            assert!(
                (id_pos - id_neg).abs() < 1e-8,
                "Id discontinuity at Vds=0, Vgs={}: {:.3e} vs {:.3e}",
                vgs,
                id_pos,
                id_neg
            );

            let (gm_pos, gds_pos) = jfet.jacobian_partial(vgs, eps);
            let (gm_neg, gds_neg) = jfet.jacobian_partial(vgs, -eps);
            assert!(
                (gm_pos - gm_neg).abs() < 1e-6,
                "gm discontinuity at Vds=0, Vgs={}: {:.6e} vs {:.6e}",
                vgs,
                gm_pos,
                gm_neg
            );
            assert!(
                (gds_pos - gds_neg).abs() < 1e-6,
                "gds discontinuity at Vds=0, Vgs={}: {:.6e} vs {:.6e}",
                vgs,
                gds_pos,
                gds_neg
            );
        }
    }

    /// The whole-device Jacobian (channel plus both gate junctions) is the
    /// derivative of its currents, both polarities, with each junction
    /// forward and reverse.
    #[test]
    fn evaluate_jacobian_is_the_derivative_of_its_currents() {
        for (channel, vp) in [(JfetChannel::N, -2.0), (JfetChannel::P, 2.0)] {
            let mut j = Jfet::new(channel, vp, 4e-3);
            j.lambda = 0.01;
            j.n = 1.3;
            let s = j.sign();
            for &(vgs, vds) in &[
                (0.55, 1.0),
                (-1.0, 3.0),
                (0.3, -0.4),
                (-0.5, -1.2),
                (0.6, 0.05),
            ] {
                let (vgs, vds) = (s * vgs, s * vds);
                let (_, _, jac) = j.evaluate(vgs, vds);
                let h = 1e-7;
                let fd = |dg: f64, dd: f64| {
                    let (a0, a1, _) = j.evaluate(vgs + dg, vds + dd);
                    let (b0, b1, _) = j.evaluate(vgs - dg, vds - dd);
                    ((a0 - b0) / (2.0 * h), (a1 - b1) / (2.0 * h))
                };
                let (did_dvgs, dig_dvgs) = fd(h, 0.0);
                let (did_dvds, dig_dvds) = fd(0.0, h);
                for (k, (an, num)) in jac
                    .iter()
                    .zip([did_dvgs, did_dvds, dig_dvgs, dig_dvds])
                    .enumerate()
                {
                    assert!(
                        (an - num).abs() <= 1e-5 * num.abs().max(1e-9),
                        "{channel:?} ({vgs}, {vds}) jac[{k}]: {an:e} vs {num:e}"
                    );
                }
            }
        }
    }

    /// A Parker–Skellern card exercising every law key (ngspice JFET level 2:
    /// VTO -2.5, BETA 1.6m, LAMBDA 4m, VST 50m, MVST 0.3, P 2.5, Q 1.8,
    /// Z 0.3, XI 3, MXI 0.2, PB 0.8).
    fn ps_card(channel: JfetChannel) -> Jfet {
        let vto = match channel {
            JfetChannel::N => -2.5,
            JfetChannel::P => 2.5,
        };
        let mut j = Jfet::parker_skellern(
            channel,
            vto,
            ParkerSkellern {
                beta: 1.6e-3,
                vst: 0.05,
                mvst: 0.3,
                p: 2.5,
                q: 1.8,
                z: 0.3,
                xi: 3.0,
                mxi: 0.2,
                vbi: 0.8,
            },
        );
        j.lambda = 0.004;
        j.is = 0.0;
        j
    }

    /// Drain currents of `ps_card` against ngspice-42's JFET2 (`.op` with the
    /// gate and drain on ideal sources, IS=0, gmin=0, reltol 1e-9, numdgt 17):
    /// deep subthreshold, the softplus knee, reverse triode, triode,
    /// saturation, reverse saturation. Both polarities.
    #[test]
    fn parker_skellern_matches_ngspice_jfet2() {
        let pins: [(f64, f64, f64); 6] = [
            (-2.9, 1.5, 6.930481155321672e-10),
            (-2.6, 0.04, 3.525744026332554e-08),
            (-2.5, -0.3, -7.605577844621228e-05),
            (-1.0, 0.2, 0.0006092811468108595),
            (0.0, 6.0, 0.008045275559950653),
            (-2.7, -2.0, -0.004264138791594062),
        ];
        for (channel, s) in [(JfetChannel::N, 1.0), (JfetChannel::P, -1.0)] {
            let j = ps_card(channel);
            for &(vgs, vds, id_ng) in &pins {
                let id = j.drain_current(s * vgs, s * vds);
                assert!(
                    (id - s * id_ng).abs() <= 1e-9 * id_ng.abs(),
                    "{channel:?} Vgs={vgs} Vds={vds}: {id:e} vs ngspice {:e}",
                    s * id_ng
                );
            }
        }
    }

    /// Below `vgst = -10·VST` the law is exactly zero (ngspice's extreme
    /// cut-off), with zero partials; above it, it conducts.
    #[test]
    fn parker_skellern_extreme_cutoff_is_exact_zero() {
        let j = ps_card(JfetChannel::N);
        // VST at Vds = 1 V is 0.05·1.3 = 65 mV; the edge is 0.65 V below VTO.
        assert_eq!(j.drain_current(-2.5 - 0.66, 1.0), 0.0);
        assert_eq!(j.jacobian_partial(-2.5 - 0.66, 1.0), (0.0, 0.0));
        assert!(j.drain_current(-2.5 - 0.64, 1.0) > 0.0);
        // VST = 0: the edge is VTO itself, a hard pinch-off.
        let mut hard = j;
        hard.ps = Some(ParkerSkellern {
            vst: 0.0,
            ..j.ps.unwrap()
        });
        assert_eq!(hard.drain_current(-2.5 - 1e-9, 1.0), 0.0);
        assert!(hard.drain_current(-2.5 + 1e-3, 1.0) > 0.0);
    }

    /// The Parker–Skellern Jacobian is the derivative of its current in every
    /// region and mode, both polarities, through the softplus and its linear
    /// continuation, and with VST = 0.
    #[test]
    fn parker_skellern_fd_jacobian() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let s = match channel {
                JfetChannel::N => 1.0,
                JfetChannel::P => -1.0,
            };
            for vst in [0.05, 0.0] {
                let mut j = ps_card(channel);
                j.ps = Some(ParkerSkellern {
                    vst,
                    ..j.ps.unwrap()
                });
                for &(vgs, vds) in &[
                    (-2.9, 1.5),
                    (-2.6, 0.04),
                    (-2.45, 0.3),
                    (-1.0, 0.2),
                    (-1.0, 4.0),
                    (0.0, 6.0),
                    (-2.5, -0.3),
                    (-2.7, -2.0),
                    (-1.0, -0.05),
                    (-1.0, 0.01),
                    (-1.0, -0.01),
                ] {
                    let (vgs, vds) = (s * vgs, s * vds);
                    if vst == 0.0 && j.drain_current(vgs, vds) == 0.0 {
                        continue;
                    }
                    let (gm, gds) = j.jacobian_partial(vgs, vds);
                    let h = 1e-7;
                    let fd_gm =
                        (j.drain_current(vgs + h, vds) - j.drain_current(vgs - h, vds)) / (2.0 * h);
                    let fd_gds =
                        (j.drain_current(vgs, vds + h) - j.drain_current(vgs, vds - h)) / (2.0 * h);
                    for (name, an, fd) in [("gm", gm, fd_gm), ("gds", gds, fd_gds)] {
                        assert!(
                            (an - fd).abs() <= 1e-6 * fd.abs().max(1e-12),
                            "{channel:?} vst={vst} ({vgs}, {vds}) {name}: {an:e} vs fd {fd:e}"
                        );
                    }
                }
            }
        }
    }

    /// Current and partials are continuous across the Vds = 0 mode swap.
    #[test]
    fn parker_skellern_continuity_across_vds_zero() {
        let j = ps_card(JfetChannel::N);
        let eps = 1e-9;
        for &vgs in &[0.0, -1.0, -2.4, -2.6, -2.9] {
            let (a, b) = (j.drain_current(vgs, eps), j.drain_current(vgs, -eps));
            assert!((a - b).abs() < 1e-10, "Id at Vgs={vgs}: {a:e} vs {b:e}");
            let (gm_a, gds_a) = j.jacobian_partial(vgs, eps);
            let (gm_b, gds_b) = j.jacobian_partial(vgs, -eps);
            assert!(
                (gm_a - gm_b).abs() < 1e-8,
                "gm at Vgs={vgs}: {gm_a:e} vs {gm_b:e}"
            );
            assert!(
                (gds_a - gds_b).abs() <= 1e-6 * gds_a.abs(),
                "gds at Vgs={vgs}: {gds_a:e} vs {gds_b:e}"
            );
        }
    }

    /// A FET limiter's gain-reduction element as registered for its refit:
    /// VTO −2.5, IDSS 10 mA (BETA 1.6e-3), LAMBDA 0.004, n = 1.
    fn sym_card(channel: JfetChannel) -> Jfet {
        let vp = match channel {
            JfetChannel::N => -2.5,
            JfetChannel::P => 2.5,
        };
        let mut j = Jfet::symmetric_subthreshold(
            channel,
            vp,
            1.6e-3 * 6.25,
            SymmetricSubthreshold::default(),
        );
        j.lambda = 0.004;
        j
    }

    /// The registered refit freezes the device at 300.15 K, Vt = 25.865 mV;
    /// the law's default thermal voltage is that value.
    #[test]
    fn symmetric_law_default_vt_is_25_865_mv() {
        let vt = SymmetricSubthreshold::default().vt;
        assert!((vt - 0.025865).abs() < 5e-7, "Vt = {vt}");
        assert_eq!(vt, VT_ROOM);
        assert!((SymmetricSubthreshold::default().scale() - 2.0 * vt).abs() < 1e-18);
    }

    /// As Vt → 0 the squared softplus is max(x, 0)² and the law is
    /// Shichman–Hodges exactly, triode and saturation, both polarities,
    /// for n = 1 and n = 1.3 alike; with LAMBDA.
    ///
    /// Pre-registered exclusion: within a few `nVt` of `Vgst = 0` and of
    /// `Vds = Vgst` the smooth law and SH differ by O(1) relative (G(0) =
    /// (2nVt·ln 2)² where SH is 0), so every grid point keeps `|Vgst|` and
    /// `|Vgst − Vds|` at least 1000·nVt (1.3e-4 V at Vt = 1e-7, n = 1.3);
    /// the closest point below is 0.05 V from either edge.
    #[test]
    fn symmetric_law_is_shichman_hodges_as_vt_vanishes() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let s = match channel {
                JfetChannel::N => 1.0,
                JfetChannel::P => -1.0,
            };
            let sh = {
                let mut j = Jfet::new(channel, -2.5 * s, 1.6e-3 * 6.25);
                j.lambda = 0.004;
                j
            };
            for n in [1.0, 1.3] {
                let mut j = sym_card(channel);
                j.sym = Some(SymmetricSubthreshold { n, vt: 1e-7 });
                // Conduction only: Shichman–Hodges below pinch-off is its
                // own 1e-12 leak, not a limit of this law.
                for &(vgs, vds) in &[
                    (0.0, 0.1),
                    (0.0, 5.0),
                    (-1.0, 0.2),
                    (-1.0, 4.0),
                    (-2.0, 0.3),
                    (-2.4, 0.05),
                    (-1.0, -0.2),
                    (-1.0, -4.0),
                    (-2.3, -0.1),
                ] {
                    let (vgs, vds) = (s * vgs, s * vds);
                    let (id_a, id_b) = (j.drain_current(vgs, vds), sh.drain_current(vgs, vds));
                    assert!(
                        (id_a - id_b).abs() <= 1e-9 * id_b.abs(),
                        "{channel:?} n={n} ({vgs}, {vds}): {id_a:e} vs SH {id_b:e}"
                    );
                    let (gm_a, gds_a) = j.jacobian_partial(vgs, vds);
                    let (gm_b, gds_b) = sh.jacobian_partial(vgs, vds);
                    assert!(
                        (gm_a - gm_b).abs() <= 1e-9 * gm_b.abs().max(1e-12),
                        "{channel:?} n={n} ({vgs}, {vds}) gm: {gm_a:e} vs SH {gm_b:e}"
                    );
                    assert!(
                        (gds_a - gds_b).abs() <= 1e-9 * gds_b.abs().max(1e-12),
                        "{channel:?} n={n} ({vgs}, {vds}) gds: {gds_a:e} vs SH {gds_b:e}"
                    );
                }
            }
        }
    }

    /// The Jacobian is the derivative of the current in every region, both
    /// polarities, both signs of Vds: strong conduction, the knee, and deep
    /// below pinch-off where the current is the thermal tail.
    #[test]
    fn symmetric_law_fd_jacobian() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let s = match channel {
                JfetChannel::N => 1.0,
                JfetChannel::P => -1.0,
            };
            for n in [1.0, 1.2] {
                let mut j = sym_card(channel);
                j.sym = Some(SymmetricSubthreshold {
                    n,
                    ..SymmetricSubthreshold::default()
                });
                for &(vgs, vds) in &[
                    (0.0, 5.0),
                    (-1.0, 0.2),
                    (-1.0, 4.0),
                    (-2.45, 0.3),
                    (-2.5, 0.04),
                    (-2.55, 0.02),
                    (-2.6, 0.04),
                    (-2.8, 0.1),
                    (-3.0, 1.5),
                    (-1.0, -0.05),
                    (-2.5, -0.3),
                    (-2.7, -2.0),
                    (-1.0, 0.01),
                    (-1.0, -0.01),
                ] {
                    let (vgs, vds) = (s * vgs, s * vds);
                    let (gm, gds) = j.jacobian_partial(vgs, vds);
                    let h = 1e-7;
                    let fd_gm =
                        (j.drain_current(vgs + h, vds) - j.drain_current(vgs - h, vds)) / (2.0 * h);
                    let fd_gds =
                        (j.drain_current(vgs, vds + h) - j.drain_current(vgs, vds - h)) / (2.0 * h);
                    for (name, an, fd) in [("gm", gm, fd_gm), ("gds", gds, fd_gds)] {
                        assert!(
                            (an - fd).abs() <= 1e-6 * fd.abs().max(1e-15),
                            "{channel:?} n={n} ({vgs}, {vds}) {name}: {an:e} vs fd {fd:e}"
                        );
                    }
                }
            }
        }
    }

    /// Source/drain symmetry: `Id(Vgs, Vds) = −Id(Vgd, −Vds)` exactly: the
    /// same two softplus values subtracted the other way, IEEE subtraction
    /// being exactly antisymmetric. The grid is dyadic (every Vgs, Vds and
    /// the pinch-off −2.5 are short binary fractions) so the swapped
    /// overdrive `(u_s − Vds) + Vds` is `u_s` bit for bit; on a non-dyadic
    /// grid the swap rounds once and the identity holds to an ulp instead.
    /// So there is no mode boundary at Vds = 0 and the current there is
    /// exactly 0.
    #[test]
    fn symmetric_law_is_odd_under_the_source_drain_swap() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let j = sym_card(channel);
            for &vgs in &[0.0, -1.0, -2.375, -2.5, -2.625, -3.0] {
                for &vds in &[0.0078125, 0.25, 2.0, 5.0] {
                    let (vgs, vds) = (j.sign() * vgs, j.sign() * vds);
                    let a = j.drain_current(vgs, vds);
                    let b = j.drain_current(vgs - vds, -vds);
                    assert_eq!(a, -b, "{channel:?} ({vgs}, {vds})");
                }
                assert_eq!(j.drain_current(j.sign() * vgs, 0.0), 0.0);
                // gm is 2·BETA·Vds in the triode region, so ±1e-9 V either
                // side of 0 differ by ~4·BETA·1e-9: an absolute bound.
                let (gm_a, gds_a) = j.jacobian_partial(j.sign() * vgs, 1e-9);
                let (gm_b, gds_b) = j.jacobian_partial(j.sign() * vgs, -1e-9);
                assert!(
                    (gm_a - gm_b).abs() <= 1e-10,
                    "gm at {vgs}: {gm_a:e} vs {gm_b:e}"
                );
                assert!((gds_a - gds_b).abs() <= 1e-6 * gds_a.abs(), "gds at {vgs}");
            }
        }
    }

    /// Deep below pinch-off the channel is a barrier current: with the gate
    /// fed half the drain swing (a FET limiter's gate feed) the current is
    /// `∝ sinh(Vds/(2nVt))`, odd in Vds, and the device stays a resistor:
    /// its conductance `I/(Vds·g0)` is `sinh(y)/y`, `y = Vds/(2nVt)`, a 10 %
    /// expansion at ±40 mV (n = 1), rather than switching off (Parker–
    /// Skellern at the same rest point drops to 0.13 there, Shichman–Hodges
    /// is a hard edge).
    #[test]
    fn symmetric_law_deep_subthreshold_is_a_sinh_resistor() {
        for n in [1.0, 1.3] {
            let mut j = sym_card(JfetChannel::N);
            j.sym = Some(SymmetricSubthreshold {
                n,
                ..SymmetricSubthreshold::default()
            });
            j.lambda = 0.0;
            let a = j.sym.unwrap().scale();
            let vg0 = -2.5 - 0.5; // 0.5 V below pinch-off: e^(−0.5/a) ≈ 6e-5 of the knee
            let g0 = j.jacobian_partial(vg0, 0.0).1;
            assert!(g0 > 0.0 && g0.is_finite());
            let mut ratio0 = None;
            for &vds in &[-0.04, -0.01, 0.001, 0.01, 0.04] {
                let id = j.drain_current(vg0 + vds / 2.0, vds);
                let ratio = id / (vds / a).sinh();
                let r0 = *ratio0.get_or_insert(ratio);
                assert!(
                    (ratio - r0).abs() <= 1e-3 * r0,
                    "n={n} Vds={vds}: I/sinh = {ratio:e}, at 1 mV {r0:e}"
                );
                let conductance = id / (vds * g0);
                let y = vds / a;
                let sinh_y_over_y = y.sinh() / y;
                assert!(
                    (conductance - sinh_y_over_y).abs() <= 2e-3
                        && (1.0..1.2).contains(&conductance),
                    "n={n} Vds={vds}: I/(V·g0) = {conductance}, sinh(y)/y = {sinh_y_over_y}"
                );
            }
        }
    }

    /// The `(1 + λ|Vds|)` factor has a kink at Vds = 0, but the bracket it
    /// multiplies is 0 there, so the law is C1: with λ ≠ 0, in conduction
    /// and below pinch-off, both polarities, the two one-sided difference
    /// quotients at Vds = 0 (h = 1e-7 V) straddle the analytic gds by no
    /// more than their own first-order truncation `(G''/G')·h/2` (`G''/G' ≤
    /// 1.22/(n·Vt)`, at the knee), and their mean, the central difference,
    /// equals it to second order. A jump would separate them by O(gds),
    /// independent of h; the bound detects a jump only down to its own size,
    /// so the analytic gds at ±1e-12 V is also held equal to 1e-9 relative.
    /// (First written as each quotient within 1e-6 of gds, which the
    /// truncation alone exceeds at h = 1e-7; the bound, not the claim, was
    /// wrong.)
    #[test]
    fn symmetric_law_gds_is_continuous_at_vds_zero_with_lambda() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let mut j = sym_card(channel);
            j.lambda = 0.05;
            let s = j.sign();
            let n_vt = j.sym.unwrap().n * j.sym.unwrap().vt;
            for &vgs in &[0.0, -1.0, -2.5, -2.7] {
                let vgs = s * vgs;
                let gds = j.jacobian_partial(vgs, 0.0).1;
                let h = 1e-7;
                let fwd = (j.drain_current(vgs, h) - j.drain_current(vgs, 0.0)) / h;
                let bwd = (j.drain_current(vgs, 0.0) - j.drain_current(vgs, -h)) / h;
                let central = 0.5 * (fwd + bwd);
                assert!(
                    (gds - central).abs() <= 1e-7 * gds.abs(),
                    "{channel:?} Vgs={vgs} gds {gds:e} vs central quotient {central:e}"
                );
                let truncation = 2.0 * (h / n_vt) * gds.abs();
                assert!(
                    (fwd - bwd).abs() <= truncation,
                    "{channel:?} Vgs={vgs}: 0+ {fwd:e} and 0- {bwd:e} differ by more than the \
                     truncation bound {truncation:e}: a jump in gds"
                );
                // What Newton consumes: the analytic gds a hair either side of
                // 0 (no truncation model needed; G''·1e-12 is ~1e-10 relative).
                let gds_p = j.jacobian_partial(vgs, 1e-12).1;
                let gds_m = j.jacobian_partial(vgs, -1e-12).1;
                assert!(
                    (gds_p - gds_m).abs() <= 1e-9 * gds.abs(),
                    "{channel:?} Vgs={vgs}: analytic gds at 0+ {gds_p:e} vs 0- {gds_m:e}"
                );
            }
        }
    }

    /// No cut-off edge: far below pinch-off the current is small, positive
    /// in the drain's direction, finite, and smaller still further down;
    /// at absurd gate voltages the softplus underflows to exactly 0 with
    /// no NaN.
    #[test]
    fn symmetric_law_has_no_cutoff_edge() {
        let j = sym_card(JfetChannel::N);
        let (i1, i2, i3) = (
            j.drain_current(-3.0, 1.0),
            j.drain_current(-4.0, 1.0),
            j.drain_current(-6.0, 1.0),
        );
        assert!(i1 > i2 && i2 > i3 && i3 > 0.0, "{i1:e} {i2:e} {i3:e}");
        assert!(i1 < 1e-9, "0.5 V below pinch-off: {i1:e}");
        for &(vgs, vds) in &[(-100.0, 10.0), (-100.0, -10.0), (-1000.0, 0.0), (50.0, 5.0)] {
            let id = j.drain_current(vgs, vds);
            let (gm, gds) = j.jacobian_partial(vgs, vds);
            assert!(
                id.is_finite() && gm.is_finite() && gds.is_finite(),
                "({vgs}, {vds})"
            );
        }
        assert_eq!(j.drain_current(-1000.0, 1.0), 0.0);
    }

    /// The whole-device Jacobian (channel plus gate junctions) is the
    /// derivative of its currents under the symmetric law too.
    #[test]
    fn symmetric_law_evaluate_jacobian_is_the_derivative_of_its_currents() {
        for channel in [JfetChannel::N, JfetChannel::P] {
            let mut j = sym_card(channel);
            j.n = 1.3;
            let s = j.sign();
            for &(vgs, vds) in &[
                (0.55, 1.0),
                (-1.0, 3.0),
                (0.3, -0.4),
                (-2.6, 0.05),
                (-2.5, -1.2),
            ] {
                let (vgs, vds) = (s * vgs, s * vds);
                let (_, _, jac) = j.evaluate(vgs, vds);
                let h = 1e-7;
                let fd = |dg: f64, dd: f64| {
                    let (a0, a1, _) = j.evaluate(vgs + dg, vds + dd);
                    let (b0, b1, _) = j.evaluate(vgs - dg, vds - dd);
                    ((a0 - b0) / (2.0 * h), (a1 - b1) / (2.0 * h))
                };
                let (did_dvgs, dig_dvgs) = fd(h, 0.0);
                let (did_dvds, dig_dvds) = fd(0.0, h);
                for (k, (an, num)) in jac
                    .iter()
                    .zip([did_dvgs, did_dvds, dig_dvgs, dig_dvds])
                    .enumerate()
                {
                    assert!(
                        (an - num).abs() <= 1e-5 * num.abs().max(1e-9),
                        "{channel:?} ({vgs}, {vds}) jac[{k}]: {an:e} vs {num:e}"
                    );
                }
            }
        }
    }

    #[test]
    #[should_panic(expected = "NSLOPE must be at least 1")]
    fn symmetric_law_rejects_a_sub_thermal_slope() {
        Jfet::symmetric_subthreshold(
            JfetChannel::N,
            -2.5,
            1e-2,
            SymmetricSubthreshold {
                n: 0.9,
                ..SymmetricSubthreshold::default()
            },
        );
    }
}
