# Day 27: Kalman Filter (Vector)

> From scalar intuition → full state-space belief.

Today’s goal was simple but deep:
**extend the Kalman Filter from a single state to a vector state** and actually understand what covariance means when states are coupled.

This is the point where Kalman filtering stops feeling like equations and starts feeling like **geometry + belief**.

---

## Problem Setup

We model a **1D constant-velocity system**.

**State**
$x = [ position velocity ]$

**Dynamics**
$xₖ₊₁ = F xₖ + wₖ$

**Measurement**
$zₖ = H xₖ + vₖ$

Key constraints:
- Only **position is measured**.
- Velocity is **never directly observed**.
- Noise exists in both dynamics and measurement.

## Model Details

- State transition: constant-velocity model.
- Process noise: unknown acceleration.
- Measurement noise: noisy position sensor.

Process noise covariance:

$$Q = σₐ² · [[dt⁴/4, dt³/2],[dt³/2, dt²]]$$

Measurement noise:

$$R = σ_z²$$

This is the standard textbook CV Kalman setup, no shortcuts.

## Results

### 1. Covariance Evolution (The Real Story)

**Observed behavior**
- Var(position) drops quickly, then stabilizes
- Var(velocity) starts very large, collapses sharply
- trace(`P`) → fast decay → flat steady-state

**Interpretation**
1. Initial uncertainty is high → the filter admits “I don’t know yet”
2. Position measurements indirectly inform velocity via cross-covariance
3. Model and sensor reach equilibrium → steady-state KF

That flat tail corresponds to a **steady-state Kalman filter**.

### 2. Position Estimation

- Noisy measurements are visibly scattered
- KF estimate is smooth and close to ground truth
- The filter does not chase noise

This indicates a healthy balance between Q and R.

### 3. Velocity Estimation

Velocity is **never measured directly**.

Early negative velocity estimates are expected and represent an **estimation transient**:
- Initial velocity guess is poor
- Early noisy measurements mislead the filter
- Covariance shrinks → estimate corrects

This behavior is physically correct and expected.

## Sanity Checklist

- Filter converges.
- Covariance remains bounded.
- Hidden state (velocity) inferred.
- No divergence.
- Physically interpretable.

## Key Takeaway

Kalman filtering is **not signal smoothing**. It is maintaining a **belief over system state** under uncertainty.

- Control assumes state.

- Estimation provides belief.

- Robotics works only when both cooperate.

## Files

```
day27_kalman_vector/
├── kalman_vector.py
├── results/
│   ├── day27_kf_vector_timeseries.png
│   └── day27_kf_vector_cov_trace.png
└── README.md
```

---