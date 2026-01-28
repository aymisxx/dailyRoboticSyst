# Day 28: Covariance as Geometry

## Why this day matters

Up to now, covariance looked like a scary matrix that just *existed* inside Kalman filters.

Today’s goal was different:

> **Stop treating covariance as algebra.  
> Start seeing it as geometry.**

Once covariance becomes a *shape in space*, estimation stops feeling magical and starts feeling engineered.

---

## Core intuition

- The **mean** represents where we believe the system state is.
- The **covariance** represents *how uncertain* that belief is.
- In 2D, covariance naturally visualizes as an **ellipse**.

This ellipse is not decoration, it is the estimator’s belief.

- Size → magnitude of uncertainty.  
- Orientation → correlation between state variables.  
- Shape → confidence along different directions.  

Kalman filtering is fundamentally about **reshaping this uncertainty geometry over time**.

## What this visualization shows

This project plots:

- the state mean.  
- the **1σ covariance ellipse**.
- the **2σ covariance ellipse**.

Both ellipses:

- share the same orientation (same covariance structure).  
- differ only by scale (confidence level).

The rotation of the ellipse comes directly from **off-diagonal covariance terms**, representing correlated uncertainty between state dimensions.

## Why this matters in robotics

Understanding covariance geometrically explains:

- why different sensors complement each other.  
- how prediction stretches uncertainty.  
- how measurements collapse it.  
- why bad models cause divergence.  
- why Kalman gain is directional, not scalar.  

Sensor fusion is not averaging. It is **intersecting uncertainty geometry**.

## Implementation details

- Eigenvalue decomposition is used to extract:
  - principal uncertainty directions.
  - corresponding confidence magnitudes.
- Ellipses are plotted using standard deviations (1σ, 2σ).
- Output figures are automatically saved for reproducibility.

## Output

The generated figure is saved in:

```
results/day28_covariance_ellipse.png
```

This plot visually captures the estimator’s belief as a geometric object in state space.

## Takeaway

> **Kalman filtering is not about estimating states.  
> It is about managing uncertainty geometry.**

Once this clicks, EKF, SLAM, and sensor fusion stop feeling mysterious and start feeling inevitable.

---