# Companion Models for Circuit Simulation

Mathematical derivations of companion models for discretizing reactive components in MNA-based circuit simulation. Primary reference: David Yeh, *A Detailed Digital Model of the Fender Bassman Push-Pull Power Amp* (Stanford, 2009).

---

## 1. Trapezoidal (Backward Differentiation) Companion Model

### 1.1 Derivation from Trapezoidal Rule

The capacitor constitutive equation relates voltage and current:

$$i(t) = C \frac{dv}{dt}$$

Integrating from $t_{n-1}$ to $t_n$ with time step $T = t_n - t_{n-1}$:

$$\int_{t_{n-1}}^{t_n} i(t) \, dt = C \int_{t_{n-1}}^{t_n} \frac{dv}{dt} \, dt = C(v_n - v_{n-1})$$

Applying the trapezoidal rule to the left-hand side:

$$\frac{T}{2}(i_n + i_{n-1}) = C(v_n - v_{n-1})$$

Solving for $i_n$:

$$i_n = \frac{2C}{T}(v_n - v_{n-1}) - i_{n-1}$$

### 1.2 Companion Model Form

Rearranging into Norton equivalent form $i_n = g_c v_n + J_n$:

$$i_n = \underbrace{\frac{2C}{T}}_{g_c} v_n + \underbrace{\left(-\frac{2C}{T}v_{n-1} - i_{n-1}\right)}_{J_n}$$

Or equivalently:

$$i_n = g_c v_n - (g_c v_{n-1} + i_{n-1})$$

Define the **history current**:

$$I_{hist} = g_c v_{n-1} + i_{n-1}$$

Then:

$$\boxed{i_n = g_c v_n - I_{hist}}$$

### 1.3 History Update Equation

The companion model requires updating the history term for the next timestep. From the original discretization:

$$I_{hist}[n] = \frac{2C}{T}v_n + i_n = g_c v_n + i_n$$

Since $i_n = g_c v_n - I_{hist}[n-1]$:

$$I_{hist}[n] = g_c v_n + (g_c v_n - I_{hist}[n-1]) = 2g_c v_n - I_{hist}[n-1]$$

**Standard update form:**

$$\boxed{I_{hist}[n] = g_c v_n + i_n = 2g_c v_n - I_{hist}[n-1]}$$

### 1.4 Matrix Form (DK Method)

For MNA, the trapezoidal rule produces:

$$A \mathbf{v}_n = \mathbf{b}_n$$

where:
- $\mathbf{A} = \frac{2\mathbf{C}}{T} + \mathbf{G}$ (forward matrix)
- $\mathbf{A}_{neg} = \frac{2\mathbf{C}}{T} - \mathbf{G}$ (history matrix)

The RHS for timestep $n$:

$$\mathbf{b}_n = \mathbf{A}_{neg} \mathbf{v}_{n-1} + 2\mathbf{w} + \mathbf{N}_i \mathbf{i}_{nl,n-1} + \mathbf{s}_n$$

where $\mathbf{w}$ represents independent source history contributions.

---

## 2. Bilinear Transform Companion Model

### 2.1 The $s \to z$ Mapping

The bilinear transform maps the Laplace domain to the $z$-domain:

$$s = \frac{2}{T} \cdot \frac{z - 1}{z + 1} = \frac{2}{T} \cdot \frac{1 - z^{-1}}{1 + z^{-1}}$$

This is derived from trapezoidal integration and provides:
- **Stability**: Maps the left-half $s$-plane inside the unit circle
- **Frequency warping**: Nonlinear relationship between analog and digital frequencies

### 2.2 Application to Series R-C Networks

For a series R-C impedance:

$$Z(s) = R + \frac{1}{sC} = \frac{1 + sRC}{sC}$$

Applying the bilinear transform:

$$Z(z) = R + \frac{T}{2C} \cdot \frac{z + 1}{z - 1} = R + \frac{T}{2C} \cdot \frac{1 + z^{-1}}{1 - z^{-1}}$$

Define companion parameters:
- $g_c = \frac{2C}{T}$ (companion conductance)
- $r_c = \frac{1}{g_c} = \frac{T}{2C}$ (companion resistance)

The discrete admittance $Y(z) = 1/Z(z)$ can be written as:

$$Y(z) = \frac{1}{R + r_c \frac{1 + z^{-1}}{1 - z^{-1}}} = \frac{g_c(1 - z^{-1})}{g_c R (1 - z^{-1}) + (1 + z^{-1})}$$

For implementation in MNA, we treat the series combination as a single admittance with history term.

### 2.3 Pre-Warping for Frequency Accuracy

The bilinear transform warps frequency:

$$\omega_a = \frac{2}{T} \tan\left(\frac{\omega_d T}{2}\right)$$

To match a specific corner frequency $\omega_c$ exactly:

$$\omega_c' = \frac{2}{T} \tan\left(\frac{\omega_c T}{2}\right)$$

**Pre-warped R-C time constant:**

$$(RC)' = \frac{1}{\omega_c'} = \frac{T/2}{\tan(\omega_c T/2)}$$

For a series R-C input network, pre-warp either R or C:

$$C_{pre} = C \cdot \frac{\omega_c T/2}{\tan(\omega_c T/2)}$$

---

## 3. Backward Euler Companion Model

### 3.1 Derivation

Backward Euler uses the approximation:

$$\frac{dv}{dt} \approx \frac{v_n - v_{n-1}}{T}$$

For a capacitor:

$$i_n = C \frac{v_n - v_{n-1}}{T} = \underbrace{\frac{C}{T}}_{g_{be}} v_n - \underbrace{\frac{C}{T} v_{n-1}}_{J_n}$$

### 3.2 Comparison with Trapezoidal

| Property | Backward Euler | Trapezoidal |
|----------|---------------|-------------|
| Companion conductance | $g_c = C/T$ | $g_c = 2C/T$ |
| History term | $I_{hist} = g_c v_{n-1}$ | $I_{hist} = g_c v_{n-1} + i_{n-1}$ |
| Local truncation error | $O(T^2)$ | $O(T^3)$ |
| Stability | A-stable (damping) | A-stable (no damping) |
| Frequency response | Low-pass characteristic | Exact at all frequencies |

### 3.3 When to Use Backward Euler

**Use trapezoidal** for:
- General circuit simulation (default)
- Preserving frequency response accuracy
- Audio-rate reactive components

**Use backward Euler** for:
- **Suppressing numerical oscillations** (ringing in inductive circuits)
- **Very stiff systems** where trapezoidal ringing is problematic
- First timestep after DC operating point (some simulators)
- Circuits with discontinuities (switching events)

The numerical damping of backward Euler:

$$|z| = \left|\frac{1}{1 + sT}\right| < 1 \quad \text{for all } \text{Re}(s) > 0$$

suppresses oscillations at the cost of accuracy.

---

## 4. Implementation Notes

### 4.1 The Critical Rule

> **A and A_neg must use the SAME G**

For trapezoidal discretization:

```
A     = (2*C/T) + G
A_neg = (2*C/T) - G   // NOT: (2*C/T) - G_dc
```

The matrix $\mathbf{G}$ must include:
- Resistor conductances ($1/R$)
- Companion conductances ($g_c = 2C/T$)
- Any other linear conductances

**Common bug**: Building $\mathbf{A}_{neg}$ from a DC-only $\mathbf{G}$ that excludes companion conductances. This breaks trapezoidal symmetry and causes incorrect frequency response.

### 4.2 History Term Initialization

At $t = 0$, the history term must be consistent with the DC operating point.

**For capacitors** (treated as open circuit at DC):

$$I_{hist}[0] = g_c V_{DC}$$

where $V_{DC}$ is the capacitor voltage from DC analysis. Since $i_{C,DC} = 0$:

$$I_{hist}[0] = g_c V_{DC} + 0 = g_c V_{DC}$$

**For inductors** (treated as short circuit at DC):

$$V_{hist}[0] = g_m I_{DC}$$

where $g_m = 2L/T$ and $I_{DC}$ is the inductor current.

### 4.3 Sample Rate Change Handling

When sample rate changes from $f_{s,old}$ to $f_{s,new}$:

1. **Recompute** $g_c = 2C/T_{new}$
2. **Rebuild** $\mathbf{A}$ and $\mathbf{A}_{neg}$ with new $T$
3. **Precompute** $\mathbf{S} = \mathbf{A}^{-1}$
4. **Recompute** $\mathbf{K} = \mathbf{N}_v \mathbf{S} \mathbf{N}_i$
5. **Scale** history terms proportionally:

$$I_{hist,new} = I_{hist,old} \cdot \frac{T_{old}}{T_{new}} = I_{hist,old} \cdot \frac{f_{s,new}}{f_{s,old}}$$

This preserves the underlying charge/volt-second continuity.

---

## 5. Common Pitfalls

### 5.1 Sign Errors in History Terms

**Wrong:**
```rust
rhs = A * v_prev - hist;  // Sign error!
```

**Right:**
```rust
rhs = A_neg * v_prev - hist;  // hist = g_c * v_prev + i_prev
```

Or equivalently with the update form:
```rust
rhs = (2*C/T) * v_prev - G * v_prev - hist_prev;
hist_new = (2*C/T) * v_new - hist_prev;  // Update for next frame
```

### 5.2 Off-by-One in History Update Timing

**Correct sequence per sample:**
1. Solve for $\mathbf{v}_n$ using $I_{hist}[n-1]$
2. Compute $i_n$ from $\mathbf{v}_n$
3. Update $I_{hist}[n] = g_c v_n + i_n$
4. **Next iteration:** use $I_{hist}[n]$ as history

**Bug pattern:** Updating history before solving, or using stale history from two timesteps ago.

### 5.3 Unit Consistency

| Quantity | SI Unit | Common Values | Conversion |
|----------|---------|---------------|------------|
| Capacitance | Farads | 4.7 µF, 100 pF | $4.7 \times 10^{-6}$, $100 \times 10^{-12}$ |
| Inductance | Henries | 10 mH, 1 H | $10 \times 10^{-3}$ |
| Resistance | Ohms | 1 MΩ, 4.7 kΩ | $10^6$, $4.7 \times 10^3$ |
| Time step | Seconds | $T = 1/f_s$ | $22.68 \, \mu s$ @ 44.1 kHz |

**Common errors:**
- Microfarads not converted to Farads: $C = 4.7$ instead of $4.7 \times 10^{-6}$
- Kiloohms not converted to ohms: $R = 4.7$ instead of $4700$
- Hz vs rad/s: $\omega = 2\pi f$ — SPICE `.ac` uses Hz, transfer functions need rad/s

### 5.4 Numerical Overflow in History Update

For very small time steps or large capacitors, $g_c = 2C/T$ can overflow. Check:

```rust
let gc = 2.0 * c / T;
assert!(gc.is_finite(), "g_c overflow: C={}, T={}", c, T);
```

At 192 kHz with $C = 1$ F, $g_c = 384000$ S (large but finite). Use `f64` for all MNA values.

---

## References

- Yeh, D.T. (2009). *A Detailed Digital Model of the Fender Bassman Push-Pull Power Amp*. Stanford University, CCRMA.
- Kundert, K.S. (1995). *The Designer's Guide to Spice and Spectre*. Kluwer Academic.
- Ogrodzki, J. (1994). *Circuit Simulation Methods and Algorithms*. CRC Press.
