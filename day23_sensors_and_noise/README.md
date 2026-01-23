# Day 23: **Sensors & Noise**  
**Reality is noisy. Sensors are liars. Engineers cope.**

---

Up to this point, control systems often assume *clean state information*.  
That assumption is **fake**.

Real robots never see the true state.  
They see **corrupted, delayed, quantized, drifting, sometimes missing signals**.

This day is about accepting that reality, and learning to reason about it *before* touching Kalman filters.

## Core idea

A sensor is **not truth**.  
A sensor is a **physical process + electronics + noise + timing artifacts**.

Formally:

$y = h(x) + v$

Where:
- $x$ → true system state  
- $h(x)$ → measurement process  
- $v$ → noise (structured, not random chaos)

The goal is **intuition**, not filtering.

## What I simulated

A 1-D “true” motion signal is observed by multiple sensor models, each introducing a different failure mode:

### Sensor types
- **White noise** - baseline Gaussian corruption  
- **Bias + noise** - constant offset error  
- **Drift (random walk)** - slow accumulating error (IMU-style)  
- **Quantization** - limited ADC resolution  
- **Latency** - delayed but otherwise clean measurements  
- **Dropouts** - missing samples (NaNs)  
- **Outliers** - rare but catastrophic spikes  
- **Combo sensor** - realistic mix of all the above

Each sensor is evaluated visually and statistically.

## Results (high-level)

### Signal comparison
- Bias shifts the signal consistently.
- Drift slowly diverges over time.
- Latency preserves shape but destroys timing.
- Outliers are rare but violent.
- Dropouts look harmless unless you need continuity.
- Combo sensor looks *plausible*, and dangerous.

### Error distributions
- White noise → symmetric Gaussian  
- Bias → shifted distribution  
- Quantization → bounded, discrete error  
- Latency → wide, high-variance error  
- Dropouts → low RMSE, high risk  
- Outliers → heavy tails  
- Combo → everything wrong at once

## Quantitative takeaway (RMSE ≠ safety)

| Sensor | RMSE | What it *actually* means |
|------|------|--------------------------|
| White | Low | Easy to filter |
| Bias | Medium | Correctable if modeled |
| Drift | Low (short-term) | Explodes long-term |
| Quantized | Very low | Bounded but coarse |
| Latency | Very high | Control-killer |
| Dropouts | Very low | Dangerous despite good stats |
| Outliers | Medium | Estimator-breakers |
| Combo | Very high | Realistic sensor pain |

> **Low RMSE does not imply a safe control signal.**

## Why this matters for robotics

- Controllers fail because of **bad sensing**, not bad math.  
- Estimation exists to manage *uncertainty*, not just smooth noise.  
- Latency hurts more than Gaussian noise.  
- Missing data is worse than inaccurate data.  
- Outliers break assumptions silently.  

This day sets the mental foundation for:
- Probabilistic thinking  
- Bayesian estimation  
- Kalman filters  
- Sensor fusion design  

## Files

day23_sensors_and_noise/  
├── sensors_and_noise.py  
├── results/  
│   ├── sensor_signals.png  
│   ├── error_histograms.png  
│   └── telemetry.csv  
└── README.md

## What I learned

- Noise has **structure**.
- Statistics can lie.
- Latency is a first-class failure mode.
- RMSE is not a control metric.
- Sensors must be *modeled*, not trusted.

#### **Next**

**Day 24: Probabilistic thinking for robotics**  
From signals → **beliefs**.

Uncertainty stops being an annoyance.  
It becomes the state.

---