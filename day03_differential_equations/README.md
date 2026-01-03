# Day 03: Differential Equations  
### Dynamics as Time-Evolving Memory

---

## Objective

The goal of Day 03 is to understand **how systems evolve over time** using
**ordinary differential equations (ODEs)** and to simulate them numerically.

This day marks the transition from:

- **static system descriptions** to  
- **dynamic, time-evolving behavior**

## Core Idea

A differential equation defines **how a system’s state changes with time**.

If the state is known at the current instant,  
the differential equation tells us how the future unfolds.

> Differential equations are not abstract math, they are **time-unfolding system memory**.

## Systems Studied

### 1. First-Order System

$$
\dot{x}(t) = -a x(t)
$$

- Models exponential decay
- System behavior depends only on the current state
- Common in thermal systems, RC circuits, and simple damping models

**Key behavior**
- Monotonic decay
- No oscillations
- Asymptotically stable

### 2. Second-Order System

$$
\ddot{x}(t) + 2\zeta\omega_n \dot{x}(t) + \omega_n^2 x(t) = 0
$$

- Represents mechanical systems with inertia
- Captures position and velocity as system memory
- Foundation of mass–spring–damper and robotic dynamics

**Key behavior**
- Oscillatory response for low damping
- Energy gradually dissipates
- Stability governed by damping ratio and natural frequency

## Numerical Simulation

Closed-form solutions are not always practical for real systems.
Instead, numerical integration is used to simulate system evolution.

**Method used**
- Explicit Euler (first-order system)
- Semi-implicit Euler (second-order system for improved stability)

This approach mirrors how real simulators propagate dynamics step-by-step.

## Folder Structure

```
day03_differential_equations/
├── ode_dynamics.py
├── results/
│   ├── first_order_response.png
│   └── second_order_response.png
└── README.md
```

## Results

### First-Order System
- Smooth exponential decay
- No numerical instability
- Confirms expected analytical behavior

### Second-Order System
- Underdamped oscillations
- Gradual amplitude decay
- Physically consistent phase and frequency

Both simulations confirm that the numerical integration
faithfully captures the underlying system dynamics.

## Takeaway

Differential equations are the **language of dynamics**.

They form the backbone of:
- system modeling
- control design
- state estimation
- robotics and autonomous systems

From this point onward, every system will be understood
as a state evolving under differential constraints.

---