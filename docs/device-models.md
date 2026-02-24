# Device Models in melange-devices

This document describes the mathematical models for nonlinear circuit devices used in the melange-devices crate. All models are implemented with real-time constraints in mind: constant-time evaluation, no dynamic allocation, and branchless execution where possible.

## Notation

| Symbol | Meaning |
|--------|---------|
| $V_t$ | Thermal voltage, $V_t = \frac{kT}{q} \approx 25.85\,\text{mV}$ at $300\,\text{K}$ |
| $I_s$ | Saturation current (A) |
| $n$ | Ideality factor (emission coefficient) |
| $\beta$ | Current gain |
| $\mu$ | Amplification factor (tubes) |

---

## 1. Diode Model

### Shockley Equation

The fundamental diode model uses the Shockley ideal diode equation:

$$
i(v) = I_s \left( \exp\left(\frac{v}{n V_t}\right) - 1 \right)
$$

Where:
- $v$ is the voltage across the diode (anode - cathode)
- $I_s$ is the saturation current
- $n$ is the ideality factor (typically 1.0 for germanium, 1.0-2.0 for silicon)
- $V_t$ is the thermal voltage

### Newton-Raphson Jacobian

For Newton-Raphson iteration, the derivative $\frac{di}{dv}$ is:

$$
\frac{di}{dv} = \frac{I_s}{n V_t} \exp\left(\frac{v}{n V_t}\right)
$$

Or equivalently:

$$
\frac{di}{dv} = \frac{i(v) + I_s}{n V_t}
$$

### Numerical Stability

For large positive $v$, the exponential overflows. We use:

```rust
// With exp() overflow protection
let exp_term = (v / (n * Vt)).min(EXP_LIMIT).exp();
let i = Is * (exp_term - 1.0);
let g = Is / (n * Vt) * exp_term;  // conductance for Jacobian
```

### Typical Parameter Values

| Device | $I_s$ (A) | $n$ | Notes |
|--------|-----------|-----|-------|
| 1N4148 (silicon) | $2.52 \times 10^{-9}$ | 1.752 | Fast switching |
| 1N914 (silicon) | $2.52 \times 10^{-9}$ | 1.752 | Similar to 1N4148 |
| 1N34A (germanium) | $1.0 \times 10^{-6}$ | 1.0 | Vintage fuzz pedals |
| 1N4007 (rectifier) | $1.0 \times 10^{-12}$ | 1.5 | Power diode |
| 1N5819 (Schottky) | $1.0 \times 10^{-8}$ | 1.06 | Low forward drop |

### SPICE Model Mapping

```spice
.model D1N4148 D(Is=2.52n N=1.752 Rs=0.568 Ikf=0 Xti=3 Eg=1.11 Cjo=4p M=0.333 Vj=0.75 Fc=0.5 Isr=1.07n Nr=2 Bv=100 Ibv=100u)
```

| SPICE Parameter | melange Parameter | Description |
|-----------------|-------------------|-------------|
| `Is` | `Is` | Saturation current |
| `N` | `n` | Ideality factor |
| `Xti` | — | Temperature exponent (not implemented) |
| `Eg` | — | Bandgap energy (not implemented) |

---

## 2. BJT Model: Ebers-Moll

The Ebers-Moll model provides the foundation for BJT simulation. We use the **transport version** which is numerically better conditioned.

### Terminal Definitions

For an NPN transistor:
- $V_{be}$: base-emitter voltage
- $V_{bc}$: base-collector voltage
- $V_{ce} = V_{be} - V_{bc}$

### Transport Model Equations

Define the **transfer current**:

$$
I_{cc} = I_s \left( \exp\left(\frac{V_{be}}{n_f V_t}\right) - \exp\left(\frac{V_{bc}}{n_r V_t}\right) \right)
$$

Where $n_f$ and $n_r$ are forward and reverse emission coefficients (typically 1.0).

The terminal currents are:

$$
I_c = I_{cc} - \frac{I_s}{\beta_r} \left( \exp\left(\frac{V_{bc}}{n_r V_t}\right) - 1 \right)
$$

$$
I_b = \frac{I_s}{\beta_f} \left( \exp\left(\frac{V_{be}}{n_f V_t}\right) - 1 \right) + \frac{I_s}{\beta_r} \left( \exp\left(\frac{V_{bc}}{n_r V_t}\right) - 1 \right)
$$

$$
I_e = -I_{cc} - \frac{I_s}{\beta_f} \left( \exp\left(\frac{V_{be}}{n_f V_t}\right) - 1 \right)
$$

### Newton-Raphson Jacobian

For MNA stamping, we need partial derivatives with respect to $V_{be}$ and $V_{bc}$:

$$
\frac{\partial I_c}{\partial V_{be}} = \frac{I_s}{n_f V_t} \exp\left(\frac{V_{be}}{n_f V_t}\right)
$$

$$
\frac{\partial I_c}{\partial V_{bc}} = -\frac{I_s}{n_r V_t} \exp\left(\frac{V_{bc}}{n_r V_t}\right) \left(1 + \frac{1}{\beta_r}\right)
$$

$$
\frac{\partial I_b}{\partial V_{be}} = \frac{I_s}{\beta_f n_f V_t} \exp\left(\frac{V_{be}}{n_f V_t}\right)
$$

$$
\frac{\partial I_b}{\partial V_{bc}} = \frac{I_s}{\beta_r n_r V_t} \exp\left(\frac{V_{bc}}{n_r V_t}\right)
$$

### PNP Polarity

For PNP transistors, flip all voltage signs and current directions:

$$
I_c^{PNP}(V_{be}, V_{bc}) = -I_c^{NPN}(-V_{be}, -V_{bc})
$$

Implementation pattern:

```rust
let vbe = if npn { v_b - v_e } else { v_e - v_b };
let vbc = if npn { v_b - v_c } else { v_c - v_b };
// ... compute currents ...
let ic = if npn { ic_computed } else { -ic_computed };
```

### Typical Parameter Values

| Device | Type | $I_s$ (A) | $\beta_f$ | $\beta_r$ | Notes |
|--------|------|-----------|-----------|-----------|-------|
| 2N2222A | NPN | $1.0 \times 10^{-14}$ | 200 | 4 | General purpose |
| 2N3904 | NPN | $6.7 \times 10^{-15}$ | 300 | 4 | Small signal |
| 2N3906 | PNP | $1.0 \times 10^{-14}$ | 250 | 4 | Complement to 2N3904 |
| BC547 | NPN | $1.8 \times 10^{-14}$ | 330 | 5 | Audio applications |
| AC128 | PNP | $5.0 \times 10^{-12}$ | 70 | 2 | Germanium, fuzz pedals |

### SPICE Model Mapping

```spice
.model Q2N2222A NPN(Is=1.0e-14 Xti=3 Eg=1.11 Vaf=74.03 Bf=255.9 Ne=1.307 Ise=1.0e-14 Ikf=0.2841 Xtb=1.5 Br=6.092 Nc=2 Isc=0 Ikr=0 Rc=1 Cjc=7.306p Mjc=0.3416 Vjc=0.75 Fc=0.5 Cje=2.2.24p Mje=0.377 Vje=0.75 Tr=46.91n Tf=411.1p Itf=0.6 Vtf=1.7 Xtf=3 Rb=10)
```

| SPICE Parameter | melange Parameter | Description |
|-----------------|-------------------|-------------|
| `Is` | `Is` | Transport saturation current |
| `Bf` | `beta_f` | Forward current gain |
| `Br` | `beta_r` | Reverse current gain |
| `Nf` | `n_f` | Forward emission coefficient |
| `Nr` | `n_r` | Reverse emission coefficient |

---

## 3. BJT Model: Gummel-Poon (Extended)

The Gummel-Poon model extends Ebers-Moll with second-order effects critical for accurate audio modeling.

### Early Effect (Base-Width Modulation)

The Early effect causes current gain to vary with collector-base voltage due to base-width modulation:

$$
q_b = \frac{1}{2} \left( 1 + \sqrt{1 + 4 \left( \frac{I_s}{I_{kf}} \exp\left(\frac{V_{be}}{n_f V_t}\right) + \frac{I_s}{I_{kr}} \exp\left(\frac{V_{bc}}{n_r V_t}\right) \right)} \right) \cdot \frac{1}{\left(1 - \frac{V_{bc}}{V_{af}}\right)\left(1 - \frac{V_{be}}{V_{ar}}\right)}
$$

The modified transfer current:

$$
I_{cc} = \frac{I_s \left( \exp\left(\frac{V_{be}}{n_f V_t}\right) - \exp\left(\frac{V_{bc}}{n_r V_t}\right) \right)}{q_b}
$$

Where:
- $V_{af}$: Forward Early voltage (typ. 50-100 V)
- $V_{ar}$: Reverse Early voltage (typ. 1-5 V)
- $I_{kf}$: Forward knee current (high-level injection onset)
- $I_{kr}$: Reverse knee current

### Simplified Early Effect (melange implementation)

For real-time efficiency, we use a simplified form:

$$
I_{cc} = I_s \frac{\exp\left(\frac{V_{be}}{n_f V_t}\right) - \exp\left(\frac{V_{bc}}{n_r V_t}\right)}{1 + \frac{|V_{bc}|}{V_{af}} + \frac{|V_{be}|}{V_{ar}}}
$$

### High-Level Injection

When emitter current exceeds $I_{kf}$, the effective $\beta$ rolls off:

$$
\beta_{f,eff} = \frac{\beta_f}{1 + \frac{I_c}{I_{kf}}}
$$

### Jacobian Extensions

The Early effect adds terms to the Jacobian. For the forward term:

$$
\frac{\partial I_{cc}}{\partial V_{be}} = \frac{I_{cc}}{n_f V_t} - \frac{I_{cc}}{V_{ar}} \cdot \text{sgn}(V_{be})
$$

### Typical Parameter Values

| Parameter | Symbol | Typical Value | Description |
|-----------|--------|---------------|-------------|
| Forward Early voltage | $V_{af}$ | 50-200 V | Output conductance |
| Reverse Early voltage | $V_{ar}$ | 1-10 V | Reverse output conductance |
| Forward knee current | $I_{kf}$ | 0.01-10 A | High-injection onset |
| Reverse knee current | $I_{kr}$ | 0.01-1 A | Reverse high-injection |

### SPICE Model Mapping

| SPICE Parameter | melange Parameter | Description |
|-----------------|-------------------|-------------|
| `Vaf` or `Va` | `V_af` | Forward Early voltage |
| `Var` or `Vb` | `V_ar` | Reverse Early voltage |
| `Ikf` | `I_kf` | Forward knee current |
| `Ikr` | `I_kr` | Reverse knee current |

---

## 4. Vacuum Tube Triode (Koren Model)

The Norman Koren model provides accurate triode simulation with smooth transitions and physically-motivated parameters.

### Core Equations

Define the effective grid-cathode voltage:

$$
E_1 = \mu \cdot V_{gk} + V_{pk} + k_{vb} \cdot V_{pk}^2
$$

Where:
- $V_{gk}$: grid-to-cathode voltage
- $V_{pk}$: plate-to-cathode voltage
- $\mu$: amplification factor
- $k_{vb}$: empirical "kink" parameter (typically 300-1500)

The plate current:

$$
I_p = \begin{cases}
\dfrac{E_1^{ex}}{k_{g1} + k_p \cdot E_1} & \text{if } E_1 > 0 \\
0 & \text{if } E_1 \leq 0
\end{cases}
$$

Where:
- $ex$: emission exponent (typically 1.4-1.5)
- $k_{g1}$: inverse transconductance scaling
- $k_p$: linearity adjustment

### Grid Current

For positive grid voltage, include grid current:

$$
I_g = \begin{cases}
k_{g2} \cdot V_{gk}^{1.5} & \text{if } V_{gk} > 0 \\
0 & \text{if } V_{gk} \leq 0
\end{cases}
$$

### Newton-Raphson Jacobian

For $E_1 > 0$:

$$
\frac{\partial I_p}{\partial V_{gk}} = \frac{\partial I_p}{\partial E_1} \cdot \frac{\partial E_1}{\partial V_{gk}} = \frac{\partial I_p}{\partial E_1} \cdot \mu
$$

$$
\frac{\partial I_p}{\partial V_{pk}} = \frac{\partial I_p}{\partial E_1} \cdot \frac{\partial E_1}{\partial V_{pk}} = \frac{\partial I_p}{\partial E_1} \cdot (1 + 2 k_{vb} V_{pk})
$$

Where:

$$
\frac{\partial I_p}{\partial E_1} = \frac{ex \cdot E_1^{ex-1}}{k_{g1} + k_p \cdot E_1} - \frac{k_p \cdot E_1^{ex}}{(k_{g1} + k_p \cdot E_1)^2}
$$

### Typical Parameter Values

| Tube | Type | $\mu$ | $ex$ | $k_{g1}$ | $k_p$ | $k_{vb}$ | Notes |
|------|------|-------|------|----------|-------|----------|-------|
| 12AX7 | Dual triode | 100 | 1.5 | 1060 | 600 | 300 | High-gain preamp |
| 12AU7 | Dual triode | 17 | 1.5 | 1180 | 330 | 400 | Medium-mu driver |
| 12AT7 | Dual triode | 60 | 1.5 | 830 | 300 | 300 | Reverb driver |
| 6N1P | Dual triode | 35 | 1.5 | 680 | 230 | 300 | Russian equivalent |
| 6DJ8 | Dual triode | 33 | 1.5 | 330 | 120 | 1000 | Linear, low-noise |
| 6SL7 | Dual triode | 70 | 1.5 | 1400 | 720 | 300 | Vintage high-gain |
| 6SN7 | Dual triode | 20 | 1.5 | 1460 | 480 | 300 | Vintage driver |

### SPICE Model Mapping

SPICE uses the `.MODEL` card with level=2 for Koren-style models, but parameter names vary. Common mapping:

```spice
.model 12AX7 XPK(Ex=1.5 Mu=100 Kg1=1060 Kp=600 Kvb=300 Rgk=2000)
```

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| `Ex` | $ex$ | Emission exponent |
| `Mu` | $\mu$ | Amplification factor |
| `Kg1` | $k_{g1}$ | Transconductance factor |
| `Kp` | $k_p$ | Linearity factor |
| `Kvb` | $k_{vb}$ | Kink voltage factor |

---

## 5. Vacuum Tube Pentode

Pentodes add a screen grid between control grid and plate, creating additional current paths.

### Screen Grid Current

The screen grid current follows a similar form to the plate current:

$$
E_{1s} = \mu_s \cdot V_{gk} + V_{sk}
$$

$$
I_s = \begin{cases}
\dfrac{E_{1s}^{ex_s}}{k_{g1s} + k_{ps} \cdot E_{1s}} \cdot f(V_{pk}, V_{sk}) & \text{if } E_{1s} > 0 \\
0 & \text{if } E_{1s} \leq 0
\end{cases}
$$

Where $f(V_{pk}, V_{sk})$ accounts for the partition of current between screen and plate.

### Plate Current with Screen Influence

$$
I_p = I_{p,triode} \cdot \left(1 - \left(\frac{V_{pk} - V_{sk}}{V_{p,sat}}\right)^2\right)
$$

Where $V_{p,sat}$ is the saturation knee voltage.

### Simplified Pentode Model (melange)

For real-time efficiency, we use a simplified model where screen current is a fraction of the "total" cathode current:

$$
I_k = \frac{E_{1}^{ex}}{k_{g1} + k_p \cdot E_{1}} \quad (E_1 > 0)
$$

$$
I_s = \alpha_s \cdot I_k \cdot \left(1 - \frac{V_{pk} - V_{sk}}{V_{p,sat}}\right) \quad \text{(clamped to [0,1])}
$$

$$
I_p = I_k - I_s
$$

Where $\alpha_s$ is the screen current ratio (typically 0.15-0.25).

### Typical Parameter Values

| Tube | Type | $\mu$ | $ex$ | $k_{g1}$ | $k_p$ | $\alpha_s$ | Notes |
|------|------|-------|------|----------|-------|------------|-------|
| EL84 | Output | 11 | 1.5 | 370 | 120 | 0.2 | 6BQ5, push-pull |
| EL34 | Output | 11 | 1.5 | 650 | 120 | 0.18 | 6CA7, high power |
| 6L6 | Output | 8 | 1.5 | 1460 | 120 | 0.15 | Beam tetrode |
| 6V6 | Output | 10 | 1.5 | 1000 | 120 | 0.18 | Lower power 6L6 |
| EF86 | Pentode | 19 | 1.4 | 600 | 200 | 0.22 | Preamp pentode |

---

## 6. Cadmium Sulfide Photoresistor (CdS LDR)

CdS photoresistors exhibit memory effects and asymmetric response times, critical for modeling vintage optical tremolo circuits.

### Power-Law Resistance Model

The static resistance follows a power law:

$$
R_{LDR} = R_{min} + \frac{R_{max} - R_{min}}{1 + (V_{env})^{\gamma}}
$$

Where:
- $R_{min}$: minimum resistance (full illumination)
- $R_{max}$: maximum resistance (dark)
- $\gamma$: power law exponent (typically 0.5-1.5)
- $V_{env}$: envelope voltage (0 to 1, proportional to light intensity)

### Asymmetric Envelope

The envelope $V_{env}$ follows separate attack and release time constants:

$$
\tau_{eff} = \begin{cases}
\tau_{attack} & \text{if } V_{drive} > V_{env} \\
\tau_{release} & \text{if } V_{drive} \leq V_{env}
\end{cases}
$$

$$
\frac{dV_{env}}{dt} = \frac{V_{drive} - V_{env}}{\tau_{eff}}
$$

Where:
- $\tau_{attack}$: attack time constant (typically 5-50 ms)
- $\tau_{release}$: release time constant (typically 50-500 ms)
- $V_{drive}$: drive voltage from LED/lamp circuit

### Current-Voltage Relationship

Once $R_{LDR}$ is computed, the LDR behaves as a simple resistor:

$$
i(v) = \frac{v}{R_{LDR}}
$$

$$
\frac{di}{dv} = \frac{1}{R_{LDR}}
$$

### Typical Parameter Values

| Parameter | Symbol | VTL5C3 | VTL5C4 | NSL-32 | Notes |
|-----------|--------|--------|--------|--------|-------|
| $R_{min}$ | $\Omega$ | 50-100 | 100-200 | 1k | Light state |
| $R_{max}$ | $\Omega$ | 1M-10M | 500k-5M | 10M | Dark state |
| $\gamma$ | — | 1.0 | 1.0 | 0.8 | Nonlinearity |
| $\tau_{attack}$ | ms | 10-20 | 5-15 | 2-5 | Rise time |
| $\tau_{release}$ | ms | 50-200 | 30-100 | 50-100 | Fall time |

### Implementation Notes

The envelope state must be stored per-instance (not const). For sample rate $f_s$:

```rust
let alpha_attack = (-1.0 / (tau_attack * fs)).exp();
let alpha_release = (-1.0 / (tau_release * fs)).exp();

// Per-sample:
let alpha = if v_drive > v_env { alpha_attack } else { alpha_release };
v_env = alpha * v_env + (1.0 - alpha) * v_drive;

let r_ldr = r_min + (r_max - r_min) / (1.0 + v_env.powf(gamma));
let g_ldr = 1.0 / r_ldr;  // Jacobian entry
```

---

## 7. Op-Amp Models

### 7.1 Ideal Op-Amp

The ideal op-amp serves as a starting point for many circuits:

**Constraints:**
- Infinite open-loop gain: $A_{ol} = \infty$
- Infinite bandwidth
- Zero output impedance
- Infinite input impedance (zero input currents)
- Virtual short: $V_+ = V_-$ (with negative feedback)

**Implementation:**

In the DK-method framework, ideal op-amps impose a constraint equation rather than a current-voltage relationship. For an inverting configuration:

$$
V_- = V_+ = \text{(known)}
$$

This eliminates a node from the unknown vector.

### 7.2 Boyle Macromodel

The Boyle macromodel adds realistic limitations while remaining computationally tractable.

#### Differential Input Stage

Differential input voltage:

$$
V_{diff} = V_+ - V_-
$$

Input current (finite input impedance):

$$
I_+ = \frac{V_+}{R_{in}} + I_{bias} + \frac{I_{offset}}{2}
$$

$$
I_- = \frac{V_-}{R_{in}} + I_{bias} - \frac{I_{offset}}{2}
$$

#### Gain Stage

Single-pole roll-off:

$$
A(s) = \frac{A_{dc}}{1 + \frac{s}{\omega_p}}
$$

Where:
- $A_{dc}$: DC open-loop gain (typically $10^5$ to $10^6$)
- $\omega_p = 2\pi f_p$: dominant pole frequency (typically 5-50 Hz)
- GBW = $A_{dc} \cdot f_p$: gain-bandwidth product (typically 1-10 MHz)

#### Slew Rate Limiting

Maximum rate of change of internal voltage:

$$
\left|\frac{dV_{int}}{dt}\right| \leq SR
$$

Implementation uses a clamped integrator:

```rust
let dv_dt = (v_in - v_int) * gain_stage_factor;
let dv_dt_limited = dv_dt.clamp(-slew_rate, slew_rate);
v_int += dv_dt_limited * dt;
```

#### Output Stage

Output voltage and current limiting:

$$
V_{out} = \text{clamp}(V_{int} \cdot A_{stage2}, -V_{sat+}, +V_{sat-})
$$

Output current limit (short-circuit protection):

$$
I_{out} = \frac{V_{out} - V_{load}}{R_{out}} \quad \text{(clamped to } \pm I_{sc}\text{)}
$$

### Typical Parameter Values

| Parameter | Symbol | 741 | TL072 | NE5532 | Notes |
|-----------|--------|-----|-------|--------|-------|
| DC gain | $A_{dc}$ | $2 \times 10^5$ | $2 \times 10^5$ | $5 \times 10^4$ | Open-loop |
| GBW | MHz | 1.0 | 3.0 | 10 | Unity-gain bandwidth |
| Slew rate | $SR$ | 0.5 V/$\mu$s | 13 V/$\mu$s | 9 V/$\mu$s | Max dV/dt |
| $R_{in}$ | M$\Omega$ | 2.0 | $10^{12}$ | 0.3 | Differential input |
| $I_{bias}$ | nA | 80 | 0.065 | 400 | Input bias current |
| $V_{sat}$ | V | $\pm 13$ | $\pm 13.5$ | $\pm 13.5$ | Output saturation |
| $R_{out}$ | $\Omega$ | 75 | 50 | 0.3 | Output impedance |

### SPICE Model Mapping

```spice
.model uA741 OpAmp(Aol=200k GBW=1Meg Slew=0.5Meg Rin=2Meg Rout=75 Vos=0.7m Ib=80n Ios=20n)
```

---

## References

1. **Shockley, W.** (1949). "The Theory of p-n Junctions in Semiconductors." *Bell System Technical Journal*.

2. **Ebers, J.J. and Moll, J.L.** (1954). "Large-Signal Behavior of Junction Transistors." *Proceedings of the IRE*.

3. **Gummel, H.K. and Poon, H.C.** (1970). "An Integral Charge Control Model of Bipolar Transistors." *Bell System Technical Journal*.

4. **Koren, N.** (1996). "Improved Vacuum Tube Models for SPICE Simulations." *Glass Audio*.

5. **Boyle, G.R. et al.** (1974). "Macromodeling of Integrated Circuit Operational Amplifiers." *IEEE Journal of Solid-State Circuits*.

6. **Getreu, I.** (1976). *Modeling the Bipolar Transistor*. Tektronix.

7. **Ytitov, A.** (2003). "A New Approach to Simulating Guitar Amplifiers." *DAFx*.
