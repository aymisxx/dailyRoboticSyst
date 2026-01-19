# Day 19: Linear Quadratic Regulator (LQR)

> **From “place the poles and pray” to “define what matters and optimize it.”**

This day introduces the **Linear Quadratic Regulator (LQR)**, an optimal state-feedback controller that explicitly balances **performance** and **control effort**.  

Unlike pole placement, LQR does not ask *where* the poles should go.  
It asks **what I care about**, and then gives me the best possible controller.

This is where control stops being manual tuning and starts being **principled design**.

---

## System Overview

We use a classic **Mass-Spring-Damper (MSD)** system:

$$
\dot{x} = Ax + Bu
$$

with state  

$$
x = \begin{bmatrix} \text{position} \\ \text{velocity} \end{bmatrix}
$$

and control input  

$$
u = \text{applied force}
$$

The same plant is controlled using:
1. **Pole Placement (PP)**: baseline state feedback.
2. **LQR**: optimal state feedback.

This ensures a **fair, apples-to-apples comparison**.

## Control Laws

### Pole Placement
$$
u = -K_{pp} x
$$

- Poles chosen manually
- Fast response
- No regard for actuator effort

### LQR
$$
u = -K_{lqr} x
$$

LQR minimizes the infinite-horizon cost:

$$
J = \int_0^\infty \left( x^T Q x + u^T R u \right)\, dt
$$

- **Q** penalizes state deviation  
- **R** penalizes control effort  

This makes the trade-off **explicit and tunable**.

## Controller Gains

| Controller | Gain Matrix |
|----------|-------------|
| Pole Placement | `K_pp = [8.5  4.2]` |
| LQR | `K_lqr = [2.78  2.39]` |

LQR intentionally produces **smaller gains** due to effort penalization.

## Results

### Control Effort Comparison

- **Pole Placement**
  - Peak \|u\| ≈ **8.5**
  - Energy ∫u²dt ≈ **8.03**

- **LQR**
  - Peak $u$ ≈ **2.78**
  - Energy $∫u²dt$ ≈ **2.46**

**LQR uses ~3.3× less control energy**.

### State Response

- Pole Placement settles **faster**
- LQR settles **more smoothly**
- LQR produces lower peak velocity and gentler transients

This behavior is **intentional**, not a flaw.

## Interpretation (The Important Part)

Pole placement answers:
> *“How fast can I force this system to behave?”*

LQR answers:
> *“What is the best behavior, given what I value?”*

Because LQR penalizes control effort:
- It avoids aggressive actuator usage
- It trades a bit of speed for efficiency and realism
- It produces controllers that are **hardware-friendly**

This is why LQR forms the backbone of:
- MPC
- Aerospace control
- Robotics systems design

## Files

```
day19_lqr/
├── lqr_control.py
├── results/
│   ├── day19_states_*.png
│   ├── day19_control_*.png
│   └── day19_summary_*.json
└── README.md
```

## Key Takeaway

> **Optimal control is not about being aggressive.  
> It’s about being intentional.**

Day 19 marks the shift from **manual control tuning** to **cost-aware system design**, a necessary step toward real-world robotics and constrained control.

*Next: actuator saturation, limits, and why optimal controllers break in reality (Day 20).*

---