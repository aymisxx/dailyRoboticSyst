# Day 14: Proportional Control (P)

## Goal
Understand how **proportional feedback** shapes closed-loop behavior and why gain tuning is a tradeoff, not a free upgrade.

This day focuses on answering one question:
> *How hard should the controller push back when the system makes an error?*

---

## System Description
A **mass-spring-damper** system is used as the plant:

$m ẍ + c ẋ + k x = u$

State variables:
- $x₁$ = position
- $x₂$ = velocity

Output:
- $y = x₁$ (position)

The reference is a **unit step**.

## Controller
A **pure proportional controller** is applied:

$u(t) = K_p · (r − y)$

Three gains are tested:
- Kp = 2  (low gain)
- Kp = 8  (moderate gain)
- Kp = 25 (high gain)

An actuator saturation limit is included to keep the simulation physically realistic.

## Results

### 1. Closed-loop Step Response
- Increasing Kp **reduces rise time** and improves tracking.
- High Kp introduces **overshoot and oscillations**.
- Low Kp is stable but sluggish and inaccurate.

Observation:
> Faster response is paid for with reduced damping and stability margin.

---

### 2. Tracking Error
- All gains exhibit **non-zero steady-state error**.
- Higher Kp reduces steady-state error but does **not eliminate it**.
- Oscillatory error appears at high gain.

Key insight:
> Pure proportional control cannot remove steady-state error for this plant.

### 3. Control Effort
- Control effort increases significantly with Kp.
- High gain leads to large oscillatory actuation.
- Even without saturation, aggressive gains demand unrealistic actuator behavior.

Engineering takeaway:
> Performance improvements come at the cost of actuator stress.

## Core Insights

- Proportional control improves responsiveness but injects energy into the system.

- Gain tuning is a balance between:
  - speed
  - stability
  - accuracy
  - actuator limits

- P-control alone is structurally limited.

## Why This Matters
This experiment shows **why PID exists**.

P-control:
- reacts to error
- but cannot reason about accumulated bias or rate of change

The limitations observed here motivate:
- Integral action (to remove steady-state error)
- Derivative action (to shape damping)

## Conclusion
Proportional control is simple, intuitive, and powerful, but incomplete.

It stabilizes systems,
reveals tradeoffs,
and exposes the need for smarter feedback.

Day 14 sets the stage for PI, PD, and full PID control.

**Clarity > cleverness.**  
**Stability is never free.**

---