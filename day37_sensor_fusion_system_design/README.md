# Day 37: Sensor Fusion as System Design

---

## 1. What is Sensor Fusion (Systems View)

Sensor fusion is the process of combining multiple imperfect sensor measurements
to form a single, internally consistent estimate of the system state that can
be safely used by a controller.

In practice, sensor fusion is less about mathematics and more about system
design decisions:

- which sensor is trusted.
- when it is trusted.
- and how failure is handled.

The fusion architecture determines whether the robot degrades gracefully or
fails catastrophically.

## 2. Why Sensor Fusion Is Not Just EKF Math

While Kalman Filters (EKF/UKF) are commonly used for fusion, they do not solve
fundamental system-level problems such as:

- mismatched sensor rates.
- delayed measurements.
- biased sensors.
- intermittent sensor failure.

A filter only amplifies the assumptions embedded in the design. Poor design
choices lead to unstable or misleading state estimates, regardless of the
filter used.

Sensor fusion must therefore be treated as a system architecture problem first,
and an estimation problem second.

## 3. Fusion Architecture (ASCII Diagram)

The following diagram represents a common fusion structure using fast inertial
prediction and slower corrective sensing:

```
                  (fast, drifts)
   IMU (200 Hz) ------------------> Prediction
                                       |
                                       v
                                State Propagation
                                       |
   Camera (30 Hz) ---> Delayed Update ->|--> State Estimate --> Controller
                  (slow, accurate)
```

Key ideas:
- IMU provides high-rate prediction but accumulates drift.
- Camera provides lower-rate but more accurate corrections.
- Fusion reconciles rate mismatch and delay.
- Controller only sees the final state estimate.

## 4. Sensor Roles & Trust Hierarchy

Each sensor plays a distinct role in the fusion pipeline:

- **IMU**

  - High rate.
  - Low latency.
  - Drift-prone.
  - Used primarily for short-term prediction.

- **Camera / Vision**

  - Lower rate.
  - Higher latency.
  - More globally consistent.
  - Used for corrective updates.

The fusion system must ensure that no single sensor dominates beyond its
reliability window.

## 5. Failure Scenario: Camera Dropout

Scenario:

- Camera data becomes unavailable for ~2 seconds.
- IMU continues providing measurements.

System behavior:

- State propagation continues using IMU prediction.
- Estimation uncertainty grows over time.
- Controller receives increasingly uncertain state estimates.
- When camera returns, delayed corrections realign the estimate.

A well-designed fusion system degrades gracefully during the dropout and
recovers without instability when corrections resume.

## 6. Key Takeaway

Sensor fusion is fundamentally about trust management under uncertainty.

Filters do not fix bad assumptions.
Architecture determines robustness.
Understanding failure modes matters more than tuning equations.

---