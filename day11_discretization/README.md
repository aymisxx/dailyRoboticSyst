# Day 11: Discretization (Continuous → Discrete Systems)

> Real controllers do not operate in continuous time.  
> They wake up every $Δt$ seconds.

This day focuses on **discretizing continuous-time dynamical systems** and understanding how sampling affects stability, dynamics, and numerical behavior.

---

## Objective

- Convert a continuous-time system into a discrete-time representation
- Understand how **sampling time** affects system behavior
- Compare **exact discretization (ZOH)** with **numerical approximation (Forward Euler)**
- Visualize stability using **discrete eigenvalues and the unit circle**

## System Studied

**Mass-Spring-Damper (MSD)** system  

State vector:
$$
x = \begin{bmatrix} q \\ \dot q \end{bmatrix}
$$

Continuous-time dynamics:
$$
\dot x = A x + B u
$$

$$
A =
\begin{bmatrix}
0 & 1 \\
-4 & -0.4
\end{bmatrix},
\quad
B =
\begin{bmatrix}
0 \\
1
\end{bmatrix}
$$

Parameters:
- m = 1.0
- c = 0.4
- k = 4.0

## Discretization Methods

### 1. Zero-Order Hold (Exact Discretization)

Assumes input is held constant over each sampling interval.

$$
A_d = e^{A \Delta t}
$$

$$
B_d = \int_0^{\Delta t} e^{A\tau} B \, d\tau
$$

Implemented using an **augmented matrix exponential**, preserving stability and dynamics.

### 2. Forward Euler (Approximate)

First-order numerical approximation:

$$
x_{k+1} = (I + A\Delta t)x_k + B\Delta t\,u_k
$$

Simple, but can distort damping, shift phase, and destabilize systems for larger Δt.

## Simulation Setup

- Sampling time: Δt = 0.02 s  
- Simulation horizon: 10 s  
- Input: unit step force (ZOH)  
- Initial state: x₀ = [1, 0]ᵀ  

Continuous-time reference simulated using **RK4** with ZOH input.

## Results

### Eigenvalue Mapping

Continuous-time eigenvalues:

$$
\lambda_c = -0.2 \pm j1.989975
$$

Discrete-time eigenvalues:

- **ZOH**:
$$
\lambda_d = 0.995219 \pm j0.039630
$$

- **Euler**:
$$
\lambda_d = 0.996000 \pm j0.039799
$$

All discrete eigenvalues lie **inside the unit circle**, confirming discrete-time stability.

### Time-Domain Behavior

- ZOH discretization closely matches the continuous-time response
- Forward Euler shows:
  - slightly larger oscillations
  - phase mismatch
  - accumulated numerical error

Velocity plots amplify these differences, highlighting discretization effects more clearly than position alone.

### Stability Interpretation

- Continuous-time stability (Re(λ) < 0) maps to discrete-time stability (|λ| < 1)
- Sampling maps the left-half plane to the **interior of the unit circle**
- Discretization method determines *where* inside the unit circle the poles land

## Files

```
day11_discretization/
├── discretization_msd.py
├── results/
│   ├── trajectories_position_ct_vs_dt.png
│   ├── trajectories_velocity_ct_vs_dt.png
│   └── discrete_eigs_unit_circle.png
```

## Key Takeaways

- Discretization is not a formality — it shapes system behavior
- ZOH preserves continuous dynamics; Euler only approximates them
- Stability must be checked **after discretization**
- Sampling time is a design parameter, not a constant

This step is foundational for digital control, estimation, MPC, and real embedded systems.

---