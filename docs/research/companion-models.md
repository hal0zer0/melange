# Companion Models for Reactive Component Discretization

Research document for AI agent consumption covering mathematical derivations, implementation formulas, and practical considerations for discretizing reactive components (capacitors and inductors) in circuit simulation using companion models.

**Primary References:**
- Yeh, D.T. (2009). *A Detailed Digital Model of the Fender Bassman Push-Pull Power Amp*. Stanford University, CCRMA.
- Kundert, K.S. (1995). *The Designer's Guide to Spice and Spectre*. Kluwer Academic.
- Ogrodzki, J. (1994). *Circuit Simulation Methods and Algorithms*. CRC Press.

---

## 1. Trapezoidal Rule Companion Model Derivation

### 1.1 Mathematical Foundation

The trapezoidal rule is derived from approximating the integral of a function using the area of a trapezoid:

$$\int_{t_{n-1}}^{t_n} f(t) \, dt \approx \frac{T}{2}\left[f(t_n) + f(t_{n-1})\right]$$

where $T = t_n - t_{n-1}$ is the time step.

### 1.2 Capacitor Discretization

**Constitutive equation:**
$$i_C(t) = C \frac{dv_C}{dt}$$

**Integrate both sides from $t_{n-1}$ to $t_n$:**
$$\int_{t_{n-1}}^{t_n} i_C(t) \, dt = C \int_{t_{n-1}}^{t_n} \frac{dv_C}{dt} \, dt = C(v_{C,n} - v_{C,n-1})$$

**Apply trapezoidal rule to LHS:**
$$\frac{T}{2}(i_{C,n} + i_{C,n-1}) = C(v_{C,n} - v_{C,n-1})$$

**Solve for $i_{C,n}$:**
$$i_{C,n} = \frac{2C}{T}(v_{C,n} - v_{C,n-1}) - i_{C,n-1}$$

### 1.3 Norton Equivalent Form

Rearrange into the standard companion model form $i = g_c v + J$:

$$i_{C,n} = \underbrace{\frac{2C}{T}}_{g_c} v_{C,n} + \underbrace{\left(-\frac{2C}{T}v_{C,n-1} - i_{C,n-1}\right)}_{J_n}$$

Define the **companion conductance** and **history current**:

| Symbol | Definition | Units |
|--------|-----------|-------|
| $g_c$ | $2C/T$ | Siemens (S) |
| $I_{hist}$ | $g_c v_{C,n-1} + i_{C,n-1}$ | Amperes (A) |

**Final companion model equation:**
$$\boxed{i_{C,n} = g_c v_{C,n} - I_{hist}}$$

### 1.4 Thevenin Equivalent Form

The same model can be expressed as a Thevenin equivalent (voltage source in series with resistance):

$$v_{C,n} = r_c i_{C,n} + V_{hist}$$

where:
- $r_c = 1/g_c = T/(2C)$ (companion resistance)
- $V_{hist} = v_{C,n-1} + r_c i_{C,n-1}$ (history voltage)

**Norton vs Thevenin:**
- Use **Norton** for nodal analysis (MNA) — injects current into nodes
- Use **Thevenin** for mesh analysis — easier for series combinations

---

## 2. Backward Euler Companion Model

### 2.1 Derivation

Backward Euler uses a first-order backward difference:

$$\frac{dv}{dt} \approx \frac{v_n - v_{n-1}}{T}$$

**For a capacitor:**
$$i_{C,n} = C \frac{v_{C,n} - v_{C,n-1}}{T}$$

**Companion parameters:**
- $g_c = C/T$ (companion conductance)
- $I_{hist} = (C/T) v_{C,n-1} = g_c v_{C,n-1}$ (history current)

**Final form:**
$$\boxed{i_{C,n} = g_c v_{C,n} - I_{hist}}$$

### 2.2 Comparison with Trapezoidal Rule

| Property | Backward Euler | Trapezoidal Rule |
|----------|---------------|------------------|
| Companion conductance | $g_c = C/T$ | $g_c = 2C/T$ |
| History term | $I_{hist} = g_c v_{n-1}$ | $I_{hist} = g_c v_{n-1} + i_{n-1}$ |
| Local truncation error | $O(T^2)$ | $O(T^3)$ |
| Stability | A-stable | A-stable |
| Numerical damping | Yes (|z| < 1) | No (|z| = 1) |
| Frequency warping | Significant at high f | Minimal |

### 2.3 When to Use Each Method

**Use Trapezoidal Rule (default):**
- Audio-frequency circuits (preserves frequency response)
- Circuits where accuracy is paramount
- Most analog modeling applications

**Use Backward Euler:**
- Suppressing numerical oscillations in stiff circuits
- Circuits with inductors (trapezoidal can ring)
- Immediately after switching events or discontinuities
- First timestep after DC operating point (some simulators)

**Numerical damping property:**
Backward Euler maps the entire left-half s-plane inside the unit circle:
$$|z| = \left|\frac{1}{1 + sT}\right| < 1 \quad \text{for all } \text{Re}(s) > 0$$

This suppresses oscillations at the cost of accuracy.

---

## 3. Bilinear Transform and Frequency Warping

### 3.1 The s-to-z Mapping

The bilinear transform maps continuous-time (Laplace) to discrete-time (z-domain):

$$s = \frac{2}{T} \cdot \frac{z - 1}{z + 1} = \frac{2}{T} \cdot \frac{1 - z^{-1}}{1 + z^{-1}}$$

**Properties:**
1. **Stability preservation**: Maps LHP s-plane to interior of unit circle in z-plane
2. **No aliasing**: Entire analog frequency axis maps to digital frequency axis
3. **Frequency warping**: Nonlinear relationship between analog and digital frequencies

### 3.2 Frequency Warping Derivation

Substitute $s = j\omega_a$ and $z = e^{j\omega_d T}$:

$$j\omega_a = \frac{2}{T} \cdot \frac{e^{j\omega_d T} - 1}{e^{j\omega_d T} + 1} = \frac{2}{T} \cdot j\tan\left(\frac{\omega_d T}{2}\right)$$

**Warping relationship:**
$$\boxed{\omega_a = \frac{2}{T}\tan\left(\frac{\omega_d T}{2}\right)}$$

where:
- $\omega_a$ = analog frequency (rad/s)
- $\omega_d$ = digital frequency (rad/s)

### 3.3 Pre-Warping for Critical Frequencies

To match a specific corner frequency $\omega_c$ exactly:

1. **Compute pre-warped analog frequency:**
$$\omega_c' = \frac{2}{T}\tan\left(\frac{\omega_c T}{2}\right)$$

2. **Adjust component values:**

For an R-C network with target $\omega_c = 1/(RC)$:

$$C_{pre} = C \cdot \frac{\omega_c T/2}{\tan(\omega_c T/2)}$$

or equivalently:

$$R_{pre} = R \cdot \frac{\omega_c T/2}{\tan(\omega_c T/2)}$$

**Pre-warping factor:**
$$k_{warp} = \frac{\omega_c T/2}{\tan(\omega_c T/2)} = \frac{\pi f_c/f_s}{\tan(\pi f_c/f_s)}$$

### 3.4 Example: Series R-C Input Network

For a series R-C with impedance:
$$Z(s) = R + \frac{1}{sC}$$

Apply bilinear transform:
$$Z(z) = R + \frac{T}{2C} \cdot \frac{1 + z^{-1}}{1 - z^{-1}}$$

Define:
- $g_c = 2C/T$ (companion conductance)
- $r_c = T/(2C)$ (companion resistance)

The discrete admittance is:
$$Y(z) = \frac{g_c(1 - z^{-1})}{g_c R (1 - z^{-1}) + (1 + z^{-1})}$$

For implementation, treat as a single admittance with appropriate history terms.

---

## 4. History Term Initialization and Update

### 4.1 History Term Definition

The history term encapsulates all past state information needed for the companion model.

**Trapezoidal rule (capacitor):**
$$I_{hist}[n] = g_c v_{C,n} + i_{C,n}$$

**Backward Euler (capacitor):**
$$I_{hist}[n] = g_c v_{C,n}$$

### 4.2 Update Equations

**Trapezoidal update (recursive form):**
$$\boxed{I_{hist}[n] = 2g_c v_{C,n} - I_{hist}[n-1]}$$

**Backward Euler update:**
$$\boxed{I_{hist}[n] = g_c v_{C,n}}$$

### 4.3 Initialization at DC Operating Point

At $t = 0$, the history term must be consistent with the DC solution.

**For capacitors (open circuit at DC):**
$$i_{C,DC} = 0$$
$$I_{hist}[0] = g_c v_{C,DC} + i_{C,DC} = g_c v_{C,DC}$$

**For inductors (short circuit at DC):**
$$v_{L,DC} = 0$$
$$V_{hist}[0] = r_m i_{L,DC} + v_{L,DC} = r_m i_{L,DC}$$

where $r_m = 2L/T$ is the inductor companion resistance.

### 4.4 Implementation Algorithm

```
// Initialization (after DC analysis)
for each capacitor:
    I_hist = gc * v_capacitor_dc

for each inductor:
    V_hist = rm * i_inductor_dc

// Per-sample processing
for each timestep:
    // 1. Build RHS using history terms
    rhs = build_rhs(I_hist, V_hist, sources)
    
    // 2. Solve for node voltages
    v = solve(matrix, rhs)
    
    // 3. Extract component voltages/currents
    for each capacitor:
        v_c = v[pos] - v[neg]
        i_c = gc * v_c - I_hist
    
    // 4. Update history terms for NEXT timestep
    for each capacitor:
        I_hist = gc * v_c + i_c    // = 2*gc*v_c - I_hist_old
```

**Critical:** Update history terms AFTER solving, BEFORE the next timestep.

---

## 5. Capacitor and Inductor Companion Models

### 5.1 Capacitor Summary

| Parameter | Trapezoidal | Backward Euler |
|-----------|-------------|----------------|
| Constitutive | $i = C \frac{dv}{dt}$ | $i = C \frac{dv}{dt}$ |
| $g_c$ | $2C/T$ | $C/T$ |
| $I_{hist}[n]$ | $g_c v_{n-1} + i_{n-1}$ | $g_c v_{n-1}$ |
| Norton model | $i_n = g_c v_n - I_{hist}$ | $i_n = g_c v_n - I_{hist}$ |
| Update | $I_{hist}[n] = 2g_c v_n - I_{hist}[n-1]$ | $I_{hist}[n] = g_c v_n$ |

### 5.2 Inductor Derivation

**Constitutive equation:**
$$v_L(t) = L \frac{di_L}{dt}$$

**Trapezoidal discretization:**
$$v_{L,n} = \frac{2L}{T}(i_{L,n} - i_{L,n-1}) - v_{L,n-1}$$

**Companion resistance and history:**
- $r_m = 2L/T$ (companion resistance)
- $V_{hist} = r_m i_{L,n-1} + v_{L,n-1}$ (history voltage)

**Norton equivalent (current-controlled):**
Transform to current source form using $g_m = 1/r_m = T/(2L)$:

$$i_{L,n} = g_m v_{L,n} - I_{hist}$$

where $I_{hist} = -g_m V_{hist} = -g_m(r_m i_{L,n-1} + v_{L,n-1}) = -i_{L,n-1} - g_m v_{L,n-1}$

### 5.3 Inductor Summary

| Parameter | Trapezoidal | Backward Euler |
|-----------|-------------|----------------|
| Constitutive | $v = L \frac{di}{dt}$ | $v = L \frac{di}{dt}$ |
| $r_m$ | $2L/T$ | $L/T$ |
| $g_m$ | $T/(2L)$ | $T/L$ |
| $V_{hist}$ | $r_m i_{n-1} + v_{n-1}$ | $r_m i_{n-1}$ |
| Thevenin model | $v_n = r_m i_n - V_{hist}$ | $v_n = r_m i_n - V_{hist}$ |
| Update | $V_{hist}[n] = 2r_m i_n - V_{hist}[n-1]$ | $V_{hist}[n] = r_m i_n$ |

### 5.4 MNA Stamps

**Capacitor between nodes p and q:**

G matrix contribution:
```
    p      q
p [  gc   -gc  ]
q [ -gc    gc  ]
```

RHS contribution at timestep n:
```
p:  +I_hist
q:  -I_hist
```

**Inductor between nodes p and q:**

G matrix contribution:
```
    p      q
p [  gm   -gm  ]
q [ -gm    gm  ]
```

RHS contribution:
```
p:  -I_hist
q:  +I_hist
```

where $I_{hist}$ for inductors is defined such that $i = g_m v - I_{hist}$.

---

## 6. Numerical Stability Considerations

### 6.1 A-Stability

A numerical method is **A-stable** if it produces bounded solutions for all stable continuous-time systems.

| Method | A-Stable | Damping |
|--------|----------|---------|
| Forward Euler | No | N/A |
| Backward Euler | Yes | Artificial |
| Trapezoidal | Yes | None |

### 6.2 Trapezoidal Ringing

Trapezoidal rule can produce **undamped numerical oscillations** when:
1. Simulating circuits with inductors
2. Rapidly changing inputs (step responses)
3. Very large time steps relative to circuit time constants

**Mitigation strategies:**
- Use Backward Euler for the first few timesteps after a discontinuity
- Use Gear's method (higher-order BDF) for stiff circuits
- Reduce timestep if oscillations are problematic

### 6.3 Stiff Systems

A circuit is **stiff** when it contains widely varying time constants:

$$\frac{\tau_{max}}{\tau_{min}} \gg 1$$

**Example:** Power supply with:
- Large filter capacitor: $\tau_1 = RC = 1$ s
- Fast switching transient: $\tau_2 = 1$ µs

**Handling stiff systems:**
1. Use Backward Euler (adds numerical damping)
2. Use adaptive timestepping
3. Use implicit methods with higher stability regions

### 6.4 Numerical Overflow

For very small time steps or large reactive components:

$$g_c = \frac{2C}{T} \rightarrow \infty \text{ as } T \rightarrow 0$$

**Practical limits:**
- At $f_s = 192$ kHz, $T = 5.2$ µs
- For $C = 1$ F: $g_c = 384,000$ S (large but finite in f64)
- For $C = 1$ pF: $g_c = 3.84 \times 10^{-7}$ S (small but nonzero)

**Implementation check:**
```rust
let gc = 2.0 * c / T;
assert!(gc.is_finite() && gc > 0.0, "Invalid companion conductance");
```

---

## 7. Sample Rate Change Handling

### 7.1 Problem Statement

When sample rate changes from $f_{s,old}$ to $f_{s,new}$:
- Time step changes: $T_{old} = 1/f_{s,old}$ → $T_{new} = 1/f_{s,new}$
- Companion conductances change: $g_c \propto 1/T$
- System matrices must be rebuilt
- History terms must be preserved consistently

### 7.2 Matrix Recomputation

**Step 1: Recompute companion parameters:**
```
g_c,new = 2C / T_new
```

**Step 2: Rebuild system matrices:**
```
A_new = (2*C/T_new) + G
A_neg_new = (2*C/T_new) - G
S_new = inverse(A_new)
K_new = N_v * S_new * N_i
```

**⚠️ CRITICAL RULE:**
Both $A$ and $A_{neg}$ must use the **SAME** $\mathbf{G}$ matrix:
```
A     = (2*C/T) + G   // Forward matrix
A_neg = (2*C/T) - G   // History matrix uses SAME G
```

Common bug: Building $A_{neg}$ from a DC-only $G$ that excludes companion conductances.

### 7.3 History Term Scaling

When sample rate changes, history terms must be scaled to preserve underlying physical quantities (charge for capacitors, flux for inductors).

**For capacitors:**
$$I_{hist,new} = I_{hist,old} \cdot \frac{T_{old}}{T_{new}} = I_{hist,old} \cdot \frac{f_{s,new}}{f_{s,old}}$$

**Derivation:**
The history term $I_{hist} = g_c v + i$ is proportional to charge rate. Since $g_c \propto 1/T$, scaling by $T_{old}/T_{new}$ preserves charge continuity.

**For inductors:**
$$V_{hist,new} = V_{hist,old} \cdot \frac{T_{old}}{T_{new}}$$

### 7.4 Implementation: Sample Rate Change Handler

```rust
struct ReactiveComponent {
    c: f64,              // Capacitance
    gc: f64,             // Companion conductance (2*C/T)
    i_hist: f64,         // History current
    v_prev: f64,         // Previous voltage
}

impl ReactiveComponent {
    fn set_sample_rate(&mut self, fs_new: f64, fs_old: f64) {
        let T_old = 1.0 / fs_old;
        let T_new = 1.0 / fs_new;
        
        // Scale history term
        self.i_hist *= T_old / T_new;
        
        // Recompute companion conductance
        self.gc = 2.0 * self.c / T_new;
    }
    
    fn update(&mut self, v_new: f64) -> f64 {
        // Compute current
        let i_new = self.gc * v_new - self.i_hist;
        
        // Update history for next timestep
        self.i_hist = 2.0 * self.gc * v_new - self.i_hist;
        
        i_new
    }
}
```

### 7.5 Full System Sample Rate Change

```rust
fn on_sample_rate_change(&mut self, fs_new: f64) {
    let fs_old = self.sample_rate;
    let T_old = 1.0 / fs_old;
    let T_new = 1.0 / fs_new;
    
    // 1. Scale all history terms
    for cap in &mut self.capacitors {
        cap.i_hist *= T_old / T_new;
    }
    for ind in &mut self.inductors {
        ind.v_hist *= T_old / T_new;
    }
    
    // 2. Rebuild system matrices with new T
    let twoC_over_T = self.c_matrix * (2.0 / T_new);
    self.a_matrix = &twoC_over_T + &self.g_matrix;
    self.a_neg_matrix = &twoC_over_T - &self.g_matrix;
    
    // 3. Recompute inverse and kernel
    self.s_matrix = self.a_matrix.inverse();
    self.k_matrix = &self.n_v * &self.s_matrix * &self.n_i;
    
    // 4. Store new sample rate
    self.sample_rate = fs_new;
}
```

---

## 8. The Critical Rule: A and A_neg Must Use the Same G

This is the single most important implementation rule for trapezoidal discretization.

### 8.1 Statement of the Rule

> **Both $A = \frac{2C}{T} + G$ and $A_{neg} = \frac{2C}{T} - G$ must use the SAME $G$ matrix.**

The matrix $G$ must include:
- Resistor conductances ($1/R$)
- Capacitor companion conductances ($g_c = 2C/T$)
- Inductor companion conductances ($g_m = T/(2L)$)
- Any other linear conductances

### 8.2 Why This Matters

The trapezoidal rule requires **symmetry** between the forward and backward steps. Using different G matrices breaks this symmetry and causes:

1. Incorrect frequency response
2. DC gain errors
3. Numerical instability
4. Transient response mismatch with SPICE

### 8.3 Common Bug Pattern

**Wrong:**
```rust
// Build G without companion conductances
let G_dc = build_g_without_companion_conductances();

// A includes companion conductances
let A = (2.0 * &C / T) + build_g_with_companion_conductances();

// A_neg uses DC G (BUG!)
let A_neg = (2.0 * &C / T) - G_dc;  // WRONG!
```

**Right:**
```rust
// Build G once, including everything
let G = build_g();  // Includes resistors AND companion conductances

// Both use same G
let twoC_over_T = &C * (2.0 / T);
let A = &twoC_over_T + &G;
let A_neg = &twoC_over_T - &G;  // Same G!
```

### 8.4 Verification Test

To verify correct implementation:

1. **DC test:** Apply DC input, verify output matches SPICE DC operating point
2. **AC test:** Small-signal frequency sweep, verify magnitude/phase match SPICE
3. **Transient test:** Step response, verify settling time and overshoot match SPICE

If $A$ and $A_{neg}$ use different G matrices, the AC response will typically show:
- Low-frequency gain error
- Incorrect corner frequencies
- Phase deviation at high frequencies

---

## 9. Implementation Reference

### 9.1 Capacitor Companion Model (Rust)

```rust
/// Trapezoidal companion model for a capacitor
pub struct CapacitorCompanion {
    /// Capacitance in Farads
    capacitance: f64,
    /// Sample period in seconds
    t: f64,
    /// Companion conductance gc = 2*C/T
    gc: f64,
    /// History current I_hist = gc*v_prev + i_prev
    i_hist: f64,
}

impl CapacitorCompanion {
    pub fn new(capacitance: f64, sample_rate: f64) -> Self {
        let t = 1.0 / sample_rate;
        let gc = 2.0 * capacitance / t;
        Self {
            capacitance,
            t,
            gc,
            i_hist: 0.0,
        }
    }
    
    /// Get companion conductance for MNA stamp
    pub fn conductance(&self) -> f64 {
        self.gc
    }
    
    /// Get history current for RHS
    pub fn history_current(&self) -> f64 {
        self.i_hist
    }
    
    /// Compute current given voltage
    pub fn current(&self, voltage: f64) -> f64 {
        self.gc * voltage - self.i_hist
    }
    
    /// Update history term for next timestep
    /// Call this AFTER solving for the new voltage
    pub fn update(&mut self, voltage: f64, current: f64) {
        // I_hist[n] = gc*v[n] + i[n] = 2*gc*v[n] - I_hist[n-1]
        self.i_hist = self.gc * voltage + current;
    }
    
    /// Initialize from DC operating point
    pub fn init_dc(&mut self, voltage_dc: f64) {
        // At DC: i = 0, so I_hist = gc * v_dc
        self.i_hist = self.gc * voltage_dc;
    }
    
    /// Handle sample rate change
    pub fn set_sample_rate(&mut self, fs_new: f64) {
        let t_new = 1.0 / fs_new;
        let t_old = self.t;
        
        // Scale history term
        self.i_hist *= t_old / t_new;
        
        // Recompute companion conductance
        self.t = t_new;
        self.gc = 2.0 * self.capacitance / t_new;
    }
}
```

### 9.2 Inductor Companion Model (Rust)

```rust
/// Trapezoidal companion model for an inductor
pub struct InductorCompanion {
    /// Inductance in Henries
    inductance: f64,
    /// Sample period in seconds
    t: f64,
    /// Companion resistance rm = 2*L/T
    rm: f64,
    /// Companion conductance gm = T/(2*L)
    gm: f64,
    /// History voltage V_hist = rm*i_prev + v_prev
    v_hist: f64,
}

impl InductorCompanion {
    pub fn new(inductance: f64, sample_rate: f64) -> Self {
        let t = 1.0 / sample_rate;
        let rm = 2.0 * inductance / t;
        let gm = 1.0 / rm;
        Self {
            inductance,
            t,
            rm,
            gm,
            v_hist: 0.0,
        }
    }
    
    /// Get companion conductance for MNA stamp (as Norton equivalent)
    pub fn conductance(&self) -> f64 {
        self.gm
    }
    
    /// Get history current for RHS
    pub fn history_current(&self) -> f64 {
        // Convert Thevenin to Norton: I_hist = -gm * V_hist
        -self.gm * self.v_hist
    }
    
    /// Compute voltage given current
    pub fn voltage(&self, current: f64) -> f64 {
        self.rm * current - self.v_hist
    }
    
    /// Update history term for next timestep
    pub fn update(&mut self, current: f64, voltage: f64) {
        // V_hist[n] = rm*i[n] + v[n] = 2*rm*i[n] - V_hist[n-1]
        self.v_hist = self.rm * current + voltage;
    }
    
    /// Initialize from DC operating point
    pub fn init_dc(&mut self, current_dc: f64) {
        // At DC: v = 0, so V_hist = rm * i_dc
        self.v_hist = self.rm * current_dc;
    }
    
    /// Handle sample rate change
    pub fn set_sample_rate(&mut self, fs_new: f64) {
        let t_new = 1.0 / fs_new;
        let t_old = self.t;
        
        // Scale history term
        self.v_hist *= t_old / t_new;
        
        // Recompute companion parameters
        self.t = t_new;
        self.rm = 2.0 * self.inductance / t_new;
        self.gm = 1.0 / self.rm;
    }
}
```

### 9.3 Unit Conversion Reference

| Quantity | SPICE Notation | SI Unit | Conversion |
|----------|---------------|---------|------------|
| Capacitance | `4.7u`, `100p` | Farads (F) | `4.7e-6`, `100e-12` |
| Inductance | `10m`, `1` | Henries (H) | `10e-3`, `1.0` |
| Resistance | `1Meg`, `4.7k` | Ohms (Ω) | `1e6`, `4700` |
| Time step | — | Seconds (s) | `T = 1/fs` |
| Frequency | `1k`, `20` | Hz | `1000`, `20` |
| Angular freq | — | rad/s | `ω = 2πf` |

**Common unit prefixes:**
| Prefix | Symbol | Multiplier |
|--------|--------|------------|
| pico | p | $10^{-12}$ |
| nano | n | $10^{-9}$ |
| micro | u (µ) | $10^{-6}$ |
| milli | m | $10^{-3}$ |
| kilo | k | $10^{3}$ |
| mega | Meg | $10^{6}$ |
| giga | G | $10^{9}$ |

---

## 10. References and Further Reading

### Primary Sources
1. **Yeh, D.T. (2009)**. *A Detailed Digital Model of the Fender Bassman Push-Pull Power Amp*. PhD Thesis, Stanford University, CCRMA.
   - The definitive reference for DK method and companion models in audio circuits.

2. **Kundert, K.S. (1995)**. *The Designer's Guide to Spice and Spectre*. Kluwer Academic.
   - SPICE implementation details, numerical methods.

3. **Ogrodzki, J. (1994)**. *Circuit Simulation Methods and Algorithms*. CRC Press.
   - General circuit simulation theory, MNA formulation.

### Related Topics
- `../dk-method.md` — DK method implementation
- `../mna-assembly.md` — MNA matrix assembly
- `../spice-translation.md` — SPICE to Rust translation guide
- `../nr-solver.md` — Newton-Raphson solver

---

*Document version: 1.0*  
*Last updated: 2026-02-23*  
*Format: AI agent research document with derivations and implementation formulas*
