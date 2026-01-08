# Day 08: State-Space Representation (Mass-Spring-Damper)

## Goal
Understand **state-space modeling** as the natural representation for dynamical systems in robotics, and see how system *states* evolve over time under different inputs.

This day focuses on *intuition*, not controller design.

---

## System Description

A classic **mass–spring–damper (MSD)** system is modeled as:

$$
m \ddot{x} + c \dot{x} + k x = u
$$

### State Definition

$$
\mathbf{x} =
\begin{bmatrix}
x \\
\dot{x}
\end{bmatrix}
$$

### State-Space Form

$$
\dot{\mathbf{x}} = A \mathbf{x} + B u
$$

where

$$
A =
\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{c}{m}
\end{bmatrix},
\quad
B =
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
$$

## Parameters Used

| Parameter | Value |
|---------|------|
| Mass (m) | 1.0 |
| Damping (c) | 0.4 |
| Spring constant (k) | 4.0 |
| Time step (dt) | 0.001 s |
| Simulation time | 10 s |
| Initial state | x = 1.0, x_dot = 0.0 |

## Experiments

### Zero Input Response (u = 0)

- System starts with initial displacement
- No external force applied
- Motion purely governed by internal dynamics

**Observed behavior**
- Underdamped oscillations
- Energy decays due to damping
- Both position and velocity converge toward zero

Final state:
- Position ≈ 0.080
- Velocity ≈ −0.241

This confirms **asymptotic stability** of the open-loop system.

### Step Input Response (u = 1 at t = 1 s)

- Constant force applied after 1 second
- System responds with oscillations around a new equilibrium

**Observed behavior**
- Sudden increase in velocity at step input
- Position overshoots, then settles
- Damping prevents sustained oscillation

Final state:
- Position ≈ 0.309
- Velocity ≈ −0.309

The steady-state shift demonstrates how **inputs directly influence state trajectories** in state-space form.

## Key Takeaways

- State-space explicitly captures **system memory**
- Both position and velocity must be tracked to predict future behavior
- Transfer functions hide internal states; state-space exposes them
- This representation naturally supports:
  - multi-state systems
  - non-zero initial conditions
  - extension to control and estimation

State-space is the **native language of robotics systems**.

## Today's Learning

A system is not defined by its output alone, it is defined by the *state that remembers the past*.

This is the foundation for feedback control, observers, and optimal control.

---