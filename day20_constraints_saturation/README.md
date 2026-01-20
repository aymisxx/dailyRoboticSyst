# Day 20: Constraints & Actuator Saturation

> *“Control does not command reality. Control negotiates with limits.”*

This experiment explores what happens when **ideal control theory collides with physical reality**.

Up to now, controllers were designed assuming unlimited actuator authority.

We study how **actuator saturation** and **integrator windup** fundamentally change closed-loop behavior, and why constraint-aware control is unavoidable in real robotic systems.

---

## Objective

- Understand how actuator limits break ideal controllers
- Observe **integrator windup** in saturated systems
- Apply **anti-windup (back-calculation)** and study its consequences
- Compare three regimes:
  1. Ideal (no limits).
  2. Saturated with windup.
  3. Saturated with anti-windup.

## Core Concept

In theory, a controller outputs:

$$
u = f(x, e, \dot e, \int e)
$$

In reality, actuators enforce:

$$
u_{\text{real}} = \mathrm{clip}(u,\ u_{\min},\ u_{\max})
$$

This clipping:
- Introduces **nonlinearity**.
- Breaks optimality guarantees.
- Invalidates linear control assumptions.
- Creates **windup** when integrators keep accumulating error that cannot be corrected.

## System Model

### Mass-Spring-Damper (MSD)

State:

$$
x = \begin{bmatrix} x \\ \dot x \end{bmatrix}
$$

Dynamics:

$$
\dot x = v
$$
$$
\dot v = \frac{1}{m}\left(u - c v - k x\right)
$$

Parameters:
- Mass $m = 1.0$
- Damping $c = 0.8$
- Spring constant $k = 4.0$

## Controller

### PID Control Law

$$
u = K_p e + K_i \int e\,dt + K_d \dot e
$$

Gains:
- $K_p = 18$
- $K_i = 8$
- $K_d = 3$

Actuator limits:

$$
u \in [-6,\ 6]
$$

## Integrator Windup

When the actuator saturates:
- The plant does **not** receive the requested input.
- Error remains non-zero.
- The integrator continues accumulating.
- The controller state becomes inconsistent with the plant.

This leads to:

- Overshoot.
- Long recovery tails.
- Oscillatory or unstable behavior.

## Anti-Windup (Back-Calculation)

Integrator state is corrected using:

$$
\dot I = e + \frac{K_{aw}}{K_i}(u_{\text{real}} - u_{\text{cmd}})
$$

Where:
- $u_{\text{cmd}}$ = unsaturated controller output.
- $u_{\text{real}}$ = saturated actuator output.
- $K_{aw}$ controls correction strength.

This prevents integrator drift, **but does not guarantee good tracking**.

## Experiments & Results

### Case 1: IDEAL (No Saturation)

- Control input spikes to **~3000**.
- Fast response.
- Low settling time.
- Astronomical control energy.

**Key insight:**  
Ideal control is mathematically elegant and physically impossible.

### Case 2: SAT (Windup)

- Actuator clamps at ±6.
- Integrator continues accumulating.
- Overshoot + long tail.
- Much lower control energy.
- Eventually settles.

**This is textbook integrator windup.**

### Case 3: SAT + Anti-Windup

- Integrator no longer lies.
- Control effort remains bounded.
- System becomes conservative.
- Reference is **never reached**.
- Rise time and settling time become undefined.

**This is not a bug. This is honesty.**

## Key Observations

### Why anti-windup can look “worse”

Anti-windup:
- Preserves internal controller consistency.
- Enforces actuator constraints strictly.
- Sacrifices performance when authority is insufficient.

> **Anti-windup does not improve tracking.  
> It improves correctness.**

The controller obeys constraints *even if it means failing the task*.

### Negative dip in anti-windup response

Early saturation forces:
- Back-calculation pushes integrator negative.
- When error reduces, integral term briefly dominates.
- Controller pushes in the opposite direction.

This is **constraint-consistent behavior**, not instability.

## Metrics Summary

| Case | Behavior |
|----|----|
| Ideal | Fast, unrealistic, infeasible |
| Saturated (windup) | Feasible, sloppy, inefficient |
| Saturated + AW | Safe, conservative, under-powered |

There is **no free lunch**.

## Why This Matters

This experiment explains why:
- PID tuning alone is insufficient.
- LQR fails under hard limits.
- Real robots require constraint-aware control.
- **Model Predictive Control (MPC)** exists.

## Conclusion

- Actuator saturation fundamentally changes system behavior.
- Integrator windup is unavoidable without protection.
- Anti-windup ensures internal consistency, not performance.
- Constraint-aware planning is mandatory for real systems.

> **Control is not about doing what you want.  
> Control is about doing what is allowed.**

---