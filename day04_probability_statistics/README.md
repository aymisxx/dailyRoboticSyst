# Day 04: Probability & Statistics for Systems

## Abstract

Real-world robotic systems never operate in noise-free conditions.  
Sensors are imperfect, environments are unpredictable, and models are approximations.

This day focuses on building **first-principles intuition for probability and statistics** as they appear in dynamical systems:
- Mean as bias / central tendency
- Variance and standard deviation as uncertainty
- Covariance as correlation between system variables
- Noise as a modeled feature, not a bug

The goal is not statistical rigor, but **systems-level understanding** required for estimation and control.

---

## Core Concepts

### 1. Mean: Central Tendency

The mean represents the expected value of a signal:

$μ = E[x]$

In systems terms:
- Mean captures **bias**
- A signal can be consistent (low variance) but biased (non-zero mean)

### 2. Variance and Standard Deviation: Uncertainty

Variance measures how much a signal spreads around its mean:

$σ² = E[(x − μ)²]$

Standard deviation:

$σ = √σ²$

In systems:
- Variance quantifies **uncertainty**
- Standard deviation is preferred because it has the same units as the signal

Low variance does **not** imply correctness: only consistency.

### 3. Covariance: Correlation Between Signals

Covariance measures how two signals vary together:

$Cov(x, y) = E[(x − μₓ)(y − μᵧ)]$

Interpretation:
- Positive covariance → signals increase together
- Negative covariance → signals move oppositely
- Zero covariance → uncorrelated (not necessarily independent)

Covariance matrices form the backbone of **state estimation**.

### 4. Noise as a Modeled Feature

Noise is not an error to be eliminated.

It arises due to:
- Sensor limitations
- Environmental disturbances
- Unmodeled dynamics

Engineering response:
**Model the noise, quantify it, and reason with it.**

## Experiment Description

A deterministic sinusoidal signal is treated as the true system output.
Gaussian noise is added to simulate sensor measurements.

A second signal is generated with partially shared noise to induce correlation.
This allows direct visualization of covariance effects.

## Implementation Summary

Signal model:

$y(t) = sin(t) + n(t)$;   $n(t)$ ~ $N(0, σ²)$

Statistics computed:
- Mean
- Variance
- Standard deviation
- Covariance matrix

Visualizations:
- Time-series (true vs noisy signal)
- Histogram with Gaussian fit
- Covariance scatter plot

## Results & Interpretation

### Noisy vs True Signal
Noise perturbs measurements while preserving underlying dynamics.

### Histogram with Gaussian Fit
Measured signal is not perfectly Gaussian.
Gaussian PDF represents a moment-matched approximation.

### Covariance Visualization
Tilted scatter structure reveals correlated behavior due to shared noise.

## Example Statistics Output

Mean: 0.1927  
Variance: 0.4762  
Std Dev: 0.6901  

Covariance Matrix:
[[0.4764, 0.0459],
 [0.0459, 0.5410]]

Diagonal terms represent individual uncertainty.  
Off-diagonal terms represent correlation.

## Key Takeaways

- Noise is unavoidable and must be modeled
- Mean captures bias
- Variance captures uncertainty
- Covariance captures interaction between variables
- Gaussian noise does not imply Gaussian signals

## File Structure

```
day04_probability_statistics/
├── README.md
├── prob_stats.py
└── results/
    ├── 01_noisy_signal_timeseries.png
    ├── 02_histogram_with_gaussian.png
    └── 03_covariance_scatter.png
```

## Forward Link

This intuition directly feeds into Bayesian estimation and Kalman filtering.
Understanding these basics prevents blind algorithmic usage later.

---