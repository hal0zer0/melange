# Companion Models (Trapezoidal Integration)

## Purpose
Convert reactive components (C, L) into equivalent conductance + current source for MNA.

## References
- Circuit Simulation Project: http://circsimproj.blogspot.com/2009/07/companion-models.html
- Pillage, Rohrer, Visweswariah. "Electronic Circuit and System Simulation Methods." McGraw-Hill (1995)

## Capacitor

### Differential Equation
```
i(t) = C · dv(t)/dt
```

### Trapezoidal Integration
```
i[n] = (2C/T)·(v[n] - v[n-1]) - i[n-1]
```

### Companion Model
```
g_eq = 2C/T = alpha·C    where alpha = 2/T
I_eq = (2C/T)·v[n-1] + i[n-1]   (history term)
i[n] = g_eq·v[n] - I_eq
```

### Equivalent Circuit
- Resistor g_eq in parallel with current source I_eq
- Current flows into positive terminal

### MNA Stamping
```
G[i,i] += g_eq, G[j,j] += g_eq
G[i,j] -= g_eq, G[j,i] -= g_eq
rhs[i] += I_eq, rhs[j] -= I_eq
```

## Inductor

### Differential Equation
```
v(t) = L · di(t)/dt
```

### Trapezoidal Integration
```
v[n] = (2L/T)·(i[n] - i[n-1]) - v[n-1]
```

### Companion Model
```
g_eq = T/(2L)
I_eq = i[n-1] + (T/2L)·v[n-1]
v[n] = (i[n] - I_eq) / g_eq
```

### Equivalent Circuit  
- Resistor g_eq in series with voltage source I_eq/g_eq
- Or: Norton equivalent with g_eq || current source

## DK Formulation (No History Vector)

### A Matrix Construction
```
A = G + alpha·C   where alpha = 2/T
A_neg = alpha·C - G
```

### History in A_neg
- **WRONG**: Store cap_history[], add to RHS separately
- **RIGHT**: A_neg·v_prev includes alpha·C·v_prev (the history)

```
rhs = A_neg · v_prev + other_terms
    = (alpha·C - G)·v_prev
    = alpha·C·v_prev - G·v_prev
```

The `alpha·C·v_prev` term IS the capacitor history contribution.

## Key Insight
Trapezoidal rule is implicit: solution at t[n] depends on itself. Companion model makes this explicit by converting differential equation to algebraic equation with equivalent conductance.

## Stability
- Trapezoidal: A-stable, 2nd order accurate, can ring
- Backward Euler: L-stable, 1st order, damps (use for problematic circuits)

## References
- http://circsimproj.blogspot.com/2009/07/companion-models.html
- Pillage & Rohrer, "Electronic Circuit and System Simulation Methods"
