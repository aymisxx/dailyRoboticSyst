# Day 16: Full PID Control

**Concept:** Full PID (Proportional-Integral-Derivative) control applied to a second-order mass-spring-damper system.

This day builds on P, PI, and PD control to show how combining all three terms shapes transient response, removes steady-state error, and improves stability.

---

## System Description

State vector:
$$
x = \begin{bmatrix} \text{position} \\ \text{velocity} \end{bmatrix}
$$

Continuous-time dynamics:
$$
\dot{x} =
\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{b}{m}
\end{bmatrix} x
+
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix} u
$$

Output:
$$
y = x_{position}
$$

## Controller

PID control law:
$$
u(t) = K_p e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}
$$

where:
- $e(t)$ = $r - y(t)$
- $K_p$: proportional gain (response speed)
- $K_i$: integral gain (steady-state error removal)
- $K_d$: derivative gain (damping and overshoot reduction)

## Simulation Setup

- Time step: `dt = 0.01 s`
- Simulation length: `10 s`
- Reference: unit step
- Integration: forward Euler
- Results saved automatically to `results/`

## Results Summary

- **Output tracking:** Fast rise, mild overshoot, stable convergence to reference  
- **Control effort:** Large initial spike due to derivative kick, then settles  
- **Tracking error:** Decays to zero due to integral action  

The system is stable and well-damped, with aggressive initial behavior typical of PID controllers tracking a step input.

## Files

```
pid_control.py
results/
├── pid_output_tracking.png
├── pid_control_effort.png
└── pid_tracking_error.png
```

## Key Takeaways

- P reacts to present error
- I eliminates steady-state error
- D improves transient damping
- PID is the first truly practical general-purpose feedback controller

This marks the transition from basic feedback to mature control design.

---