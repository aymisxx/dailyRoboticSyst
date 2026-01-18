# Day 18: Controllability

## What this day is about

Before designing *any* controller, there is one brutal question that must be answered:

> **Can this system even be controlled?**

Controllability is a **structural property** of a dynamical system.  
If a system is not controllable, **no controller** (PID, LQR, MPC, RL) can force it to reach arbitrary states, some modes are simply unreachable.

This day focuses on building **intuition + verification** for controllability in linear time-invariant (LTI) systems.

---

## System model

We consider continuous-time LTI systems of the form:

$$
\dot{x} = Ax + Bu
$$

where:
- $x \in \mathbb{R}^n$ is the state
- $u \in \mathbb{R}^m$ is the control input

## Controllability (Kalman rank condition)

A system is **controllable** if the controllability matrix

$$
\mathcal{C} = [B \; AB \; A^2B \; \dots \; A^{n-1}B]
$$

has **full rank**:

$$
\text{rank}(\mathcal{C}) = n
$$

Intuitively:
- `B` shows which states are affected **directly** by the input
- `AB, A²B, ...` show how input influence **propagates through dynamics**
- Full rank means input influence reaches **all state dimensions**

## PBH test (sanity check)

As an additional verification, the **Popov–Belevitch–Hautus (PBH) test** is used:

A system $(A,B)$ is controllable **iff** for every eigenvalue $\lambda$ of $A$:

$$
\text{rank}([\lambda I - A  B]) = n
$$

Both Kalman rank and PBH tests are implemented.

## Experiments

Two 2-state systems are evaluated:

### System 1: Controllable
- Input affects the dynamics in a way that influences **both states**
- Controllability matrix has full rank
- Reachable set is **2-D**

### System 2: Not controllable
- Input affects only one state
- The second state has **no dynamics and no input**
- Controllability matrix rank drops to 1
- Reachable set collapses to a **1-D manifold**

## Reachability intuition (simulation)

To build geometric intuition, random control sequences are applied from the zero initial state.

Final states are plotted in the $(x_1, x_2)$ plane:

- **Controllable system:** states spread in both dimensions
- **Uncontrollable system:** states lie on a line (`x₂ = 0`)

This visually demonstrates that uncontrollability manifests as a **loss of reachable dimensions**.

## Files

```
day18_controllability/
├── controllability.py
├── README.md
└── results/
    ├── controllability_report.txt
    └── reachability_scatter.png
```

## Key takeaways

- Controllability is **not** a tuning issue, it is structural.
- No controller can compensate for missing actuation paths.
- Hardware choices often determine controllability before software does.
- Reachable sets provide powerful geometric intuition.

## Why this matters

State feedback, pole placement, LQR, and MPC **all assume controllability**.

Checking controllability first prevents:
- impossible controller designs.
- misleading simulation results.
- false confidence in control performance.

This is where control design becomes **systems thinking**, not just math.

---