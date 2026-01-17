# Day 17: State Feedback (Full-State Feedback)

## Objective
Move beyond PID-style error correction and implement **full-state feedback control** using a state-space model.  
The goal is to directly shape closed-loop system behavior by **placing the system poles**.

This day answers one core question:

> *If I know the full state, can I directly command how the system behaves?*  

Yes. That’s state feedback.

---

## System Description
We consider a classic **mass-spring-damper (MSD)** system with force input:

$$
x =
\begin{bmatrix}
x_1 \\ x_2
\end{bmatrix}
=
\begin{bmatrix}
\text{position} \\ \text{velocity}
\end{bmatrix}
$$

$$
\dot{x} = Ax + Bu
$$

where:

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

Parameters used:
- Mass $m = 1.0 \, \text{kg}$
- Damping $c = 0.5 \, \text{Ns/m}$
- Stiffness $k = 2.0 \, \text{N/m}$

## Control Law

Full-state feedback is applied:

$$
u = -Kx
$$

This modifies the closed-loop dynamics to:

$$
\dot{x} = (A - BK)x
$$

The gain matrix $K$ is computed using **pole placement**.

### Desired Closed-Loop Poles
$$
\lambda = -2 \pm 2j
$$

Resulting gain:
$$
K = [6.0 \quad 3.5]
$$

## Simulation Setup

- Initial state: $x(0) = [1.0,\; 0.0]^T$
- Time horizon: 8 seconds
- Time step: 0.01 s
- Optional actuator saturation: $|u| \le 10 \, \text{N}$

Two cases are simulated:
1. **Open-loop** (no control input)
2. **Closed-loop** (state feedback)

## Results

- Open-loop system shows lightly damped oscillations.
- Closed-loop system converges rapidly with minimal overshoot.
- Initial control input is approximately **- 6 N**, consistent with $u(0) = -Kx(0)$.
- Control effort decays smoothly and does not hit saturation.

Plots are saved in the `results/` directory:
- `position_open_vs_closed.png`
- `velocity_open_vs_closed.png`
- `control_input.png`

## Notes

- NumPy emits a **DeprecationWarning** related to scalar conversion.
- This warning does **not** affect numerical correctness or results.
- It is safely ignored for this learning exercise.

## File Structure

```
day17_state_feedback/
├── README.md
├── state_feedback.py
└── results/
    ├── position_open_vs_closed.png
    ├── velocity_open_vs_closed.png
    └── control_input.png
```

## Takeaway

State feedback marks the transition from **tuning controllers** to **engineering dynamics**. Once full state is available, control becomes a design problem, not a guessing game.

---