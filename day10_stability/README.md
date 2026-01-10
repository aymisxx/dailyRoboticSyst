# Day 10: **Stability (Continuous-Time LTI)**

This day focuses on **stability analysis** of continuous-time linear time-invariant (LTI) systems using **eigenvalues**.

The goal is to understand how system behavior can be predicted *without simulation*, purely from the system matrix.

---

## System Studied

A standard **mass–spring–damper (MSD)** system modeled in state-space form:

$$
\dot{x} = A x
$$

where the state vector is:

$$
x = \begin{bmatrix} x \\ \dot{x} \end{bmatrix}
$$

and

$$
A =
\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{c}{m}
\end{bmatrix}
$$

### Parameters
- Mass $m = 1.0$
- Damping $c = 0.4$
- Spring constant $k = 4.0$

Resulting system matrix:

```
A =
 [ 0.   1. ]
 [-4.  -0.4]
```

## Eigenvalue Analysis

The eigenvalues of the system matrix are:

$$
\lambda = -0.2 \pm 1.9899j
$$

### Interpretation
- Negative real part → exponential decay
- Non-zero imaginary part → oscillatory behavior

**Stability verdict:**  
> **Asymptotically Stable**

This means all trajectories decay to the equilibrium at the origin.

## Simulation (Free Response)

To verify the eigenvalue-based prediction, the free response of the system was simulated with:

- Initial condition:  
  $$
  x(0) = [1.0,\; 0.0]^T
  $$
- No external input

### Observations
- Position and velocity oscillate
- Amplitude decays over time
- System converges to equilibrium

This behavior exactly matches the eigenvalue analysis.

The plot below confirms stability visually:

```
results/free_response_states.png
```

## Key Takeaways

- **Eigenvalues determine stability** for continuous-time LTI systems.
- Simulation is a confirmation tool, not a necessity.
- Oscillatory decay corresponds to complex eigenvalues with negative real parts.
- Stability can be assessed *before* running any time-domain simulation.

## Files

```
day10_stability/
├── stability.py
├── results/
│   └── free_response_states.png
└── README.md
```

## Why This Matters

Stability analysis is the foundation for:
- state feedback design
- pole placement
- LQR
- MPC feasibility
- diagnosing real-world system failures

From this point onward, control design becomes **shaping eigenvalues**, not guessing behavior.

---