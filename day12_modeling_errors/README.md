# Day 12: Modeling Errors
*Why models lie, and why feedback still saves you (sometimes).*

---

## Objective

The goal of this day was to **break the illusion of perfect models**.

Up to now, systems were assumed to behave exactly as described by equations.
Today’s focus is understanding what happens when **the controller believes one model**, but **the real plant is different**.

This experiment studies how **parameter uncertainty** affects:
- closed-loop stability
- transient response
- control effort (with actuator saturation)

## System Description

A standard **mass–spring–damper system** is used:

$m ẍ + c ẋ + k x = u$

State vector:
$x = [ x, ẋ ]ᵀ$

State-space form:
$ẋ = [[0, 1], [−k/m, −c/m]] x + [[0], [1/m]] u$

## Controller Design (Nominal Model)

- Controller: **state feedback**
- Design method: pole placement (Ackermann)
- Designed on **nominal parameters only**

Nominal parameters:
- m = 1.0
- c = 0.25
- k = 5.0

Desired closed-loop poles:
$λ = {−2, −3}$

Resulting gain:
K = [1.0  4.75]

The same controller is reused for all perturbed plants.

## Modeling Errors Tested

| Case | Mass | Damping | Stiffness |
|-----|------|---------|-----------|
| Nominal | 1.0 | 0.25 | 5.0 |
| Mass +20% | 1.2 | 0.25 | 5.0 |
| Damping −40% | 1.0 | 0.15 | 5.0 |
| Stiffness +30% | 1.0 | 0.25 | 6.5 |
| All-wrong | 1.2 | 0.15 | 6.5 |

Actuator saturation:
$|u| ≤ 10$

## Results

### Closed-loop Position Response

- All cases remain **stable**
- Parameter mismatch affects **settling speed and transient shape**
- Increased stiffness causes slight **undershoot**
- Reduced damping introduces more oscillatory behavior

Feedback stabilizes the system, but performance degrades as the model drifts.

### Control Effort

- Higher stiffness significantly increases peak control effort
- Reduced damping increases transient demand
- Combined errors push the controller closer to saturation

The controller work, but **pays a higher control cost** when the model lies.

## Key Observations

- Stability does not imply robustness margin
- Controllers tolerate *some* modeling error
- Performance degradation appears before instability
- Control effort is often the first real-world failure mode

## Takeaway

All models are wrong.
Feedback decides how wrong you’re allowed to be.

This experiment builds intuition for why robustness, constraints, and uncertainty must be treated seriously in real robotic systems.

## What This Prepares For

- Robust control
- Actuator saturation analysis
- Model Predictive Control
- Real-world robotic systems with uncertainty

---