# Day 26: Scalar Kalman Filter (1D)

This day introduces the **scalar Kalman filter**, extending Bayesian estimation by incorporating **system dynamics and uncertainty propagation**.

The goal is not derivation-heavy rigor, but **intuition-first understanding** of how prediction and measurement are optimally fused in time.

---

## Problem Setup

We consider a **1D state estimation** problem with noisy measurements.

### State (process) model

$$x_k = x_{k-1} + w_k$$

- Random-walk dynamics  
- $w_k$ ~ $N(0, Q)$ (process noise)

### Measurement model

$$z_k = x_k + v_k$$

- Direct noisy observation.  
- $v_k$ ~ $N(0, R)$ (measurement noise).

Both noises are assumed **Gaussian and independent**.

## Kalman Filter Algorithm (Scalar)

Each time step consists of two phases.

### 1. Prediction

$$x̂_k⁻ = x̂_{k-1}$$  

$$P_k⁻ = P_{k-1} + Q$$  

Uncertainty increases due to process noise.

### 2. Update

Kalman Gain:  

$$K_k = P_k⁻ / (P_k⁻ + R)$$

State update:  

$$x̂_k = x̂_k⁻ + K_k (z_k − x̂_k⁻)$$

Covariance update:  

$$P_k = (1 − K_k) P_k⁻$$

The Kalman gain automatically balances trust between **prediction** and **measurement** based on their uncertainties.

## Results

- The estimate converges rapidly from a poor initial guess.
- Measurement noise is smoothed without lag or bias.
- Steady-state covariance matches theoretical expectations.
- The filter remains stable and unbiased over time.

This demonstrates that **Kalman filtering is disciplined Bayesian estimation with memory**, not a complex black box.

## Files

```
day26_scalar_kalman/
├── kalman_scalar.py  
├── README.md  
└── results/  
    ├── kalman_scalar_plot.png  
    └── kalman_scalar_data.csv  
```

## Key Takeaways

- Kalman filtering is conceptually simple in scalar form.
- Uncertainty (covariance) is the central quantity, not the estimate itself.
- Kalman gain is a principled trust coefficient.
- This scalar case forms the foundation for vector Kalman filters, EKF, and sensor fusion.

---