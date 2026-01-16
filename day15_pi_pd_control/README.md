# Day 15: PI vs PD Control  
**Integral Memory vs Derivative Instinct**

---

## Overview

This experiment compares **PI** and **PD** controllers on a simple **first-order dynamical system** to build intuition around:

- steady-state error  
- transient behavior  
- control effort  
- why PID exists in the first place  

No black boxes. No libraries doing magic.  
Just equations, simulation, and consequences.

## System Model

We control a first-order plant:

$$
\dot{x} = -a x + b u
$$

where:
- $x$ is the system output  
- $u$ is the control input  
- $a, b > 0$ are known constants  

The task is to **track a step reference** $r = 1$.

## Controllers

### PI Control (Memory)

$$
u(t) = K_p e(t) + K_i \int e(t)\,dt
$$

- Accumulates past error  
- Eliminates steady-state error  
- Provides constant control effort to counteract plant decay  

### PD Control (Prediction)

$$
u(t) = K_p e(t) + K_d \frac{de(t)}{dt}
$$

- Reacts to error trends  
- Improves damping and transient response  
- **Cannot eliminate steady-state error** on its own  

## Simulation Setup

- Numerical integration: **Euler method**
- Time step: `dt = 0.01 s`
- Simulation horizon: `T = 10 s`
- Identical plant and proportional gain for fair comparison
- PD derivative initialized to avoid artificial startup spikes

## Results

### Output Response

- **PI controller** reaches the reference asymptotically with **zero steady-state error**
- **PD controller** settles below the reference due to lack of integral action

This behavior is **structural**, not a tuning flaw.

### Control Effort

- PI produces a **non-zero steady control input**, required to balance plant decay
- PD settles at a lower control effort but sacrifices tracking accuracy

The trade-off is unavoidable and visible.

## Key Takeaways

- Integral action is memory → removes bias and steady-state error  
- Derivative action is instinct → improves transient behavior  
- PD alone is insufficient for perfect tracking  
- PID exists because real systems demand both  

## Files

```
pi_pd_ctrl.py
results/
├── pi_vs_pd_output.png
└── pi_vs_pd_control_effort.png
```

## Closing Note

Control isn’t about forcing systems to obey.  
It’s about understanding what they remember, what they predict,  
and what they need to survive at equilibrium.

Day 15 was about intuition, not tuning.

---