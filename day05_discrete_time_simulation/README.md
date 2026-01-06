# Day 5: Discrete-Time Systems & Simulation

**Focus:**  
How continuous-time dynamics behave when simulated in discrete time, and why numerical stability is *not* guaranteed by physical stability.

This day marks the transition from “solving equations” to **building simulators**.

---

## Objective

The goal of this day was to:
- Understand how continuous-time systems are discretized
- Build a minimal discrete-time simulator from scratch
- Observe how the choice of time step (`dt`) affects stability
- Separate **physical stability** from **numerical stability**

The emphasis is on intuition, not optimization.

## System under study

We study the simplest stable continuous-time system:

$$
\dot{x} = -a x \quad (a > 0)
$$

This system is physically stable and converges exponentially to zero.

## Discretization method

The system is simulated using **forward Euler discretization**:

$$
x_{k+1} = x_k + dt \cdot (-a x_k) = (1 - a\,dt)\,x_k
$$

This yields a discrete-time linear system whose stability depends on the discrete-time eigenvalue:

$$
\lambda_d = 1 - a\,dt
$$

## Experiments performed

### 1. Discrete-time simulation
- Implemented a forward Euler simulation loop
- Verified behavior for small `dt`
- Confirmed agreement with the continuous-time solution

### 2. Effect of time step
Simulations were run with multiple values of `dt`:
- Small `dt` → stable, accurate decay
- Larger `dt` → oscillatory behavior
- `dt > 2/a` → numerical instability and divergence

Despite identical physics, the simulated behavior changed drastically.

### 3. Analytical stability condition
For Euler discretization, stability requires:

$$
|1 - a\,dt| < 1 \quad \Rightarrow \quad 0 < dt < \frac{2}{a}
$$

This condition is **purely numerical**.

### 4. Euler vs exact discretization
The exact discrete-time solution:

$$
x_{k+1} = e^{-a\,dt} x_k
$$

was compared with Euler discretization for large `dt`.

Result:
- Euler becomes unstable
- Exact discretization remains stable for any `dt > 0`

## Key insight

> A system can be **physically stable** but **numerically unstable**.

Discretization method and time step selection are **modeling choices**, not implementation details.

This distinction is fundamental for:
- Simulation
- Control design
- State estimation (e.g., Kalman filters)
- Reinforcement learning environments

## Files

```
day05_discrete_time_simulation/
├── notebook.ipynb   # Conceptual exploration, plots, and intuition
├── demo.py          # Minimal reproducible simulation script
└── README.md        # This file
```

## Takeaway

Numerical methods do not merely approximate dynamics, they **shape system behavior**.

Understanding this early prevents silent failures later in control, estimation, and learning-based robotic systems.

---