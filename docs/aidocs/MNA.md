# Modified Nodal Analysis (MNA)

## Purpose
Convert circuit netlist to system of linear equations: `(G + sC)·v = i`

## References
- Ho et al., "The Modified Nodal Approach to Network Analysis" (1975)
- TU Delft: https://analog-electronics.ewi.tudelft.nl/webbook/SED/

## Matrices
- **G**: Conductance matrix (resistors, companion models) - real, symmetric
- **C**: Capacitance matrix (caps, inductors as companion) - real, symmetric  
- **v**: Node voltages (unknown)
- **i**: Current sources (inputs)

## Stamping Rules

### Resistor R between nodes i,j
```
g = 1/R
G[i,i] += g, G[j,j] += g
G[i,j] -= g, G[j,i] -= g
```
Source: TU Delft Analog Electronics Webbook, "563 MNA stamps"

### Capacitor C between nodes i,j
```
C[i,i] += C, C[j,j] += C
C[i,j] -= C, C[j,i] -= C
```
Note: Actual stamping is into C matrix, NOT G. G gets companion conductance via alpha*C.

### Input Conductance (Thevenin source)
```
G[in,in] += 1/R_source  // Single diagonal term to ground
```
This is critical - without it, input voltage divider is wrong.

### Diode (nonlinear)
Not stamped in G/C. Added to nonlinear device list for DK kernel.

## Common Bugs
1. **Missing input conductance** → Wrong gain (S matrix magnitude wrong)
2. **Sign error in off-diagonals** → Negative resistances, unstable
3. **Ground node handling** → Node 0 excluded from matrix
4. **Stamping C into G directly** → Wrong frequency response

## Verification
- G should be symmetric positive semi-definite
- Diagonal entries ≥ sum of off-diagonals (diagonal dominance)
- Input node G[i,i] must include source conductance

## References
- Ho, Ruehli, Brennan. "The Modified Nodal Approach to Network Analysis." IEEE Trans. CAS (1975)
- https://analog-electronics.ewi.tudelft.nl/webbook/SED/_build/html/network_theory/NetworkTheory-1_Modified_Nodal_Analysis.html
