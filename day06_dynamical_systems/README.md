# Day 06: **What is a Dynamical System?**

A **dynamical system** is anything that has **memory**.

If the future depends only on what’s happening *right now*, it’s algebra. If the future depends on what happened *before*, it’s dynamics.

Robotics, control, estimation, reinforcement learning, all of it lives here.

---

## Core idea: State, Input, Output

Any dynamical system can be described using three roles:

- **State (`x`)**  
  The system’s internal memory.  
  It is the **minimum set of variables** required to predict future behavior.

- **Input (`u`)**  
  What we can externally apply or command.  
  Forces, torques, voltages, throttle, control signals.

- **Output (`y`)**  
  What we can measure or observe.  
  Often incomplete — which is why estimation exists.

> If you know the full state at time `t`, you don’t need the entire past.

> This is the **Markov property**.

## Canonical system form

### Continuous time
$$
\dot{x}(t) = f(x(t), u(t)) \\
y(t) = g(x(t), u(t))
$$

### Discrete time
$$
x_{k+1} = f(x_k, u_k) \\
y_k = g(x_k)
$$

Everything later, stability, control, observers, MPC, Kalman filters, RL, plugs into this structure.

## The demo: a minimal discrete-time system

The `demo.py` file implements a tiny **linear discrete-time system**:

$$
x_{k+1} = A x_k + B u_k \\
y_k = C x_k
$$

Interpretation:
- `x = [position, velocity]ᵀ` (intuition only)
- A short impulse input is applied for the first few steps
- The system then evolves freely

Only the **position** is measured as output.

## What the results show

- A brief input causes a **lasting change** in the state  
- Velocity decays over time (stable mode)
- Position converges to a steady value

In other words:

> The system **remembers** the input in position,  
> but **forgets** it in velocity.

That is the essence of a dynamical system.

## Stability intuition (1D reminder)

For a simple recurrence:
$$
x_{k+1} = a x_k
$$

- $|a| < 1$ → stable (memory fades)
- $|a| > 1$ → unstable (memory explodes)
- $|a| = 1$ → marginal (context matters)

Eigenvalues are not just math — they are **destiny indicators**.

## Why this matters for robotics

Robots:
- have memory
- are multi-state
- are partially observable
- evolve over time

State-space modeling is not a choice, it’s a necessity.

This day sets the foundation for:
- physical modeling
- control design
- state estimation
- learning-based control

## Files

```
day06/
├── README.md   # theory & intuition
└── demo.py     # minimal discrete-time system demo
```

Run:
```bash
python demo.py
```

**Day 06 takeaway:**  
If you understand *what the system remembers*, you understand the system.

---