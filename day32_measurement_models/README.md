# Day 32: Measurement Models in Robotics

## What I did today

Today focused on understanding **measurement models**, the formal link between a robot’s internal state and what sensors report.

Instead of implementing sensors immediately, the goal was to build a **correct mental model** of what sensors do *and do not* measure, and why perception errors propagate downstream into estimation, planning, and control.

This day is intentionally theory-first.

---

## Core idea

Robots do not observe state directly.

They observe **measurements**, which are functions of the state corrupted by noise, bias, and delay.

Formally:

$$z = h(x) + v$$

Where:

- `x` → true system state (hidden)
- `h(·)` → measurement model
- `z` → sensor output
- `v` → measurement noise

Perception is the act of choosing and trusting a model `h(x)`.

## State vs Measurement

| Concept | Meaning |
|------|-------|
| State (`x`) | What the robot *is* (position, velocity, orientation, etc.) |
| Measurement (`z`) | What the sensor *reports* |
| Measurement model (`h`) | How state maps to measurement |
| Noise (`v`) | Uncertainty, bias, latency, quantization |

Control and planning **never see the state**.  
They only see **beliefs derived from measurements**.

## Example: 2D Range Measurement

Assume a robot with state:

$$x = [x, y]^T$$

A range sensor measuring distance from the origin reports:

$$z = sqrt(x² + y²) + v$$

Where:

- `v ~ N(0, σ²)`

### Key observations

- The sensor cannot distinguish between different angles.
- Multiple states map to the same measurement.
- The measurement model is **nonlinear**.
- Noise increases uncertainty non-uniformly in state space.

This loss of information must be handled by the estimator.

## What sensors cannot do

Sensors:

- do **not** give truth.  
- do **not** observe full state.  
- do **not** self-validate correctness.  

They only provide partial, noisy projections of reality.

Treating sensor outputs as state is a common failure mode in robotics systems.

## Failure modes

### 1. Incorrect measurement model

If `h(x)` is wrong:
- estimators diverge.
- filters appear “unstable”.
- tuning becomes impossible.

### 2. Underestimated noise

If noise variance is underestimated:
- filters become overconfident.
- corrections overshoot.
- small errors grow catastrophically.

### 3. Ignored bias and delay

Bias and latency violate estimator assumptions and cause:
- drift.
- oscillations.
- control instability.

## Why this matters

Measurement models define **what the robot believes**.

Estimation, planning, and control performance are bounded by:
- sensor observability.
- correctness of `h(x)`.
- realism of noise assumptions.

Perception is not vision or sensors.  
It is **system design under uncertainty**.

## Key takeaway

Perception does not reveal reality.  
It constructs belief.

Bad measurement models produce confident failure.

---