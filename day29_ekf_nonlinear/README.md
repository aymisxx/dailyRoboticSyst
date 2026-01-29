# Day 29: Extended Kalman Filter (EKF)

## Why this day matters

Real robots do **not** live in linear worlds.

Motion models are nonlinear.  
Sensors are nonlinear.  
And yet, we still want uncertainty-aware estimation.

The Extended Kalman Filter (EKF) exists to bridge this gap.

> **EKF assumes the world is nonlinear, but locally honest.**

---

## Core idea

The EKF applies the Kalman filter framework to nonlinear systems by:

1. Propagating the state using **nonlinear dynamics**
2. Linearizing the system **around the current estimate**
3. Applying standard Kalman update equations on this local approximation

The filter repeats this process at every timestep.

Same Kalman logic.  
New linearization every step.

## System modeled

This example uses a deliberately simple but nonlinear setup:

### State dynamics

$$x_{k+1} = x_k + sin(x_k)·dt + w_k$$

### Measurement model

$$z_k = x_k² + v_k$$

Where:

- $w_k$ is process noise.
- $v_k$ is measurement noise.

Both the **motion** and **measurement** models are nonlinear.

## EKF mechanics used

At each timestep:

- Predict state using nonlinear dynamics
- Compute Jacobians:
  - $F = ∂f / ∂x$
  - $H = ∂h / ∂x$
- Propagate covariance.
- Compute Kalman gain.
- Update state and covariance.

Covariance is updated using the **Joseph form** for numerical stability.

## Results

The plot shows:

- True system state.  
- EKF state estimate.  

Saved output:

```
results/day29_ekf_timeseries.png
```

### Observations

- EKF rapidly converges from an incorrect initial estimate.
- Tracking is accurate when the system is locally smooth.
- Small bias appears as nonlinearity increases, expected behavior.
- No divergence or instability observed.

This demonstrates EKF’s strength **and** its limitation.

## Key insight

> **EKF works when linearization error remains small.**

As curvature increases or uncertainty grows:
- Linear approximation degrades.
- Estimation bias can increase.
- Divergence becomes possible.

This directly motivates more advanced filters such as the Unscented Kalman Filter (UKF).

## Takeaway

EKF is not magic.  
It is a disciplined approximation strategy.

When used within its assumptions, it is powerful.  
When pushed beyond them, it fails predictably.

Understanding *why* is more important than memorizing equations.

---