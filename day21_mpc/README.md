# Day 21: Why Model Predictive Control (MPC) Exists

This experiment demonstrates **why MPC is needed in real control systems**, and why simply applying an optimal controller (LQR) with actuator clipping is *not* equivalent to constrained optimal control.

---

The core idea:
> **Real systems have hard constraints. MPC is designed for them.**

## Problem Setup

We consider a **1D double integrator** system:

- **State**:  
  $x = [ position, velocity ]ᵀ$

- **Control input**:  
  u = acceleration

- **Discrete-time dynamics**:
  $xₖ₊₁ = A xₖ + B uₖ$

with a **hard actuator constraint**:
$-1 ≤ u ≤ 1$

The goal is to drive the system from an initial position of 8 to the origin while respecting actuator limits.

## Controllers Compared

### 1. LQR + Saturation (Baseline)

- Infinite-horizon optimal control
- Designed **without constraints**
- Actuator limits enforced afterward via clipping

This reflects a common real-world hack:
> *Design an optimal controller, then hope saturation doesn’t break things.*

### 2. Constrained MPC

- Finite-horizon optimization
- Actuator limits enforced **inside** the optimization
- Receding-horizon (re-plans at every timestep)

This reflects how real safety-critical systems are designed.

## Results

### Control Input

- **LQR** aggressively demands more acceleration than the actuator allows and remains saturated.
- **MPC** also uses the limits, but **backs off earlier**, anticipating future state evolution.

This is the key distinction:  
**MPC plans with feasibility in mind, LQR does not.**

### Position

- **LQR + clip** exhibits large overshoot due to accumulated kinetic energy.
- **MPC** reduces overshoot by braking earlier and respecting future constraints.

### Velocity

- LQR builds up higher velocity before correction.
- MPC limits velocity growth proactively, leading to smoother convergence.

## Key Takeaways

- LQR assumes unlimited actuation and infinite horizon.
- Actuator saturation fundamentally breaks LQR optimality.
- Clipping an LQR controller **does not make it constraint-aware**.
- MPC explicitly optimizes **over feasible trajectories**, not just instantaneous cost.

> **MPC exists because real systems must choose feasible actions, not optimal fantasies.**

## Why This Matters

This distinction is critical in:
- Autonomous vehicles
- Drones and UAVs
- Robotics manipulators
- Safety-critical aerospace systems

Whenever constraints matter (they always do), MPC provides a principled solution.

## Files

day21_mpc/  
├── mpc_code.py  
├── results/  
│   ├── day21_controls.png  
│   ├── day21_positions.png  
│   └── day21_velocities.png  
└── README.md

## Status

The Concept validated.  
The Results match control theory intuition.  
Our Day 21 objective achieved.  

Ready to move on to **Day 22**.

---