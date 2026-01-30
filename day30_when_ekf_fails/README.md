# Day 30: When EKF Fails: Overconfidence, Consistency & Estimation vs Control

This experiment demonstrates **why Extended Kalman Filters fail silently**, and why
**low RMSE does NOT imply a healthy estimator**.

The focus is not mathematical derivation, but **systems-level estimator behavior**
under nonlinearity and aggressive motion.

## Objective

- Study EKF behavior under **reasonable vs overconfident noise tuning**.
- Expose **covariance collapse** and **statistical inconsistency**.
- Show why **innovation consistency (NIS)** matters more than RMSE.
- Highlight **estimation-control coupling** under aggressive maneuvers.

## System Setup

- **Robot model:** Unicycle (nonlinear)
- **Measurements:** Range + bearing to a fixed landmark
- **Estimator:** EKF
- **Control regimes:**
  - *Mild* - smooth angular motion
  - *Aggressive* - hard turns, higher nonlinearity

Two EKFs are run in parallel:

| Filter | Description |
|------|------------|
| EKF OK | Reasonable Q / R (conservative, consistent) |
| EKF Overconfident | Q and R set too small (arrogant estimator) |

## Key Results

### 1. Covariance Trace: The Overconfidence Collapse

- The **overconfident EKF collapses its covariance almost immediately**.
- `trace(P)` stays unrealistically small regardless of motion difficulty.
- The filter internally believes large errors are impossible.

In contrast:
- The OK EKF **adapts uncertainty**.
- `trace(P)` grows during aggressive motion and shrinks when measurements help.

A healthy estimator must *breathe* with system dynamics.

### 2. RMSE vs Statistical Consistency (The Trap)

Despite similar RMSE values:

```
[mild]       mean NIS: OK ≈ 1.0 | Overconfident ≈ 23
[aggressive] mean NIS: OK ≈ 1.0 | Overconfident ≈ 23
```

- Expected NIS for 2D measurement ≈ **2**
- OK EKF ≈ **statistically consistent**
- Overconfident EKF ≫ **statistically impossible under its own assumptions**

**Interpretation:**
> The overconfident EKF is accurate by chance, not by correct uncertainty modeling.

This is a **pre-failure condition** in real robotic systems.

### 3. Aggressive Control Reveals the Truth

Under aggressive motion:
- OK EKF increases uncertainty → consistent innovation behavior
- Overconfident EKF experiences massive innovation shocks (NIS spikes up to ~200)

Estimation failure is triggered by **control-induced nonlinearity**.

This directly demonstrates **estimation-control coupling**.

### 4. Trajectories (Why Visuals Can Lie)

- Overconfident EKF trajectories may look “clean”.
- But small covariance + huge innovations = **fragile system**.
- Failure will be **sudden, not gradual**.

Real robots do not crash slowly.

## Core Insight (Day 30 Takeaway)

> **RMSE measures accuracy.  
NIS measures honesty.  
Only honest estimators are safe.**

Most real-world “control failures” are actually **estimation failures wearing control’s clothes**.

## Engineering Takeaways

- EKF does not degrade gracefully, it **snaps**.
- Overconfidence is more dangerous than noise.
- Innovation consistency must be monitored in deployed systems.
- Estimation quality determines control safety.

## What This Enables Next

- Understanding **why UKF, particle filters, and smoothing exist**.
- Better debugging of perception-control stacks.
- System-level thinking instead of filter worship.

## Files

```
day30_when_ekf_fails/
├── when_ekf_fails.py
├── results/
│   ├── day30_covtrace_mild.png
│   ├── day30_covtrace_aggressive.png
│   ├── day30_errors_mild.png
│   ├── day30_errors_aggressive.png
│   ├── day30_innovation_nis_mild.png
│   ├── day30_innovation_nis_aggressive.png
│   ├── day30_trajectories_mild.png
│   └── day30_trajectories_aggressive.png
```

## Status

- Estimation intuition built  
- Systems thinking unlocked  

Next: **Perception as a system (Day 31)**.

---