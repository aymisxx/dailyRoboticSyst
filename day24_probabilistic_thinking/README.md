# Day 24: Probabilistic Thinking for Robotics

## What this day is about

Robots do not know the true state of the world. They maintain **beliefs** about it.

This day introduces the shift from **deterministic states** to **probabilistic states**, which is foundational for estimation, sensor fusion, and Kalman filtering.

---

## Core idea
Instead of representing state as a single value:

```
x = 5.0
```

we represent it as a **probability distribution**:

```
x ~ N(μ, σ²)
```

- **Mean (μ)** → best estimate of the state  
- **Variance (σ²)** → confidence in that estimate  

The state is no longer a point, it is a belief.

## What was implemented
- Sampled a 1D Gaussian belief over robot position.
- Visualized the belief as a probability density histogram.

This plot represents **what the robot believes**, not ground truth.

## Key takeaway
- Sensor readings are noisy evidence, not truth.
- The robot’s internal state is a probability distribution.
- Uncertainty is a first-class concept in robotics.
- Estimation exists because reality is uncertain.

This intuition is required before Bayesian estimation and Kalman filtering.

---