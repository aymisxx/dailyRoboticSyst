# Day 34: IMU Intuition & Integration  
**Why IMU-only dead reckoning drifts even for a stationary robot**

## Objective

To build **systems-level intuition** for IMU behavior by demonstrating, with a minimal simulation, how:

- Gyroscope bias causes orientation drift.  
- Orientation drift corrupts gravity removal.  
- Incorrect gravity removal creates fake linear acceleration.  
- Double integration of that acceleration produces large position drift.  

All experiments are performed on a **perfectly stationary robot**.

---

## Sensor Models

This demo uses simplified but physically consistent IMU models in 2D (yaw only).

### Gyroscope model

The gyroscope measures angular velocity:

$$
\omega_{\text{meas}}(t) = \omega_{\text{true}}(t) + b_\omega + n_\omega(t)
$$

Where:
- $\omega_{\text{true}} = 0$ (robot is stationary).
- $b_\omega$ is a constant gyro bias.
- $n_\omega$ is zero-mean Gaussian noise.

Orientation estimate is obtained by integration:

$$
\hat{\theta}(t) = \int_0^t \omega_{\text{meas}}(\tau)\, d\tau
$$

Even a small constant bias produces **linearly growing orientation error**.

### Accelerometer model

The accelerometer measures **specific force**, not velocity or position:

$$\mathbf{a}_{\text{meas}}^{\text{body}} = \mathbf{R}^T(\theta)\,\mathbf{g} + \mathbf{b}_a + \mathbf{n}_a$$

Where:
- $\mathbf{g} = [0, g]^T$ is gravity.
- $\mathbf{R}(\theta)$ is the rotation from body to world.
- $\mathbf{b}_a$ is accelerometer bias.
- $\mathbf{n}_a$ is accelerometer noise.

At rest, the accelerometer primarily measures gravity.

## Gravity Removal and the Core Failure

To obtain linear acceleration in the world frame, gravity is subtracted:

$$\mathbf{a}_{\text{lin}}^{\text{world}} = \mathbf{R}(\hat{\theta})\,\mathbf{a}_{\text{meas}}^{\text{body}} - \mathbf{g}$$

If the orientation estimate $\hat{\theta}$ is wrong:

- Gravity is rotated incorrectly.
- Subtraction leaves a **horizontal gravity component**.
- This appears as real acceleration, even though the robot is stationary.

Approximate effect:

$$a_x \approx g \sin(\tilde{\theta})$$

where $\tilde{\theta}$ is the orientation error.

## Dead Reckoning Integration

Linear acceleration is integrated naively:

$$\mathbf{v}(t) = \int \mathbf{a}_{\text{lin}}(t)\, dt$$

$$\mathbf{p}(t) = \int \mathbf{v}(t)\, dt$$

Any bias or persistent error results in **quadratic position growth**.

## Experiment Design

Two cases are simulated using the **same noise realization**:

### Case A: Bias ON
- Gyro bias: **0.5 deg/s**.
- Accelerometer bias + noise present.

### Case B: Bias OFF (Control)
- Gyro bias: **0.0 deg/s**.
- Same noise and accelerometer parameters.

This isolates the effect of **gyro bias alone**.

## Results

### Case A: Bias ON

- **Yaw drift**:  
  ~15° over 30 seconds (matches $0.5 \times 30 = 15$ deg exactly).

- **Fake horizontal acceleration**:  
  Grows to ~2.5 m/s².  
  Consistent with $g \sin(15^\circ)$.

- **Position drift**:  
  ~374 meters for a stationary robot.

This drift is not numerical instability, it is a **physically inevitable outcome** of incorrect gravity removal.

### Case B: Bias OFF

- **Yaw drift**:  
  Small random walk (~0.02°), noise-only.

- **Linear acceleration**:  
  Zero-mean noisy signal, no trend.

- **Position drift**:  
  ~10 meters over 30 seconds due to double integration of noise.

Even without bias, IMU-only position estimation is **not stable long-term**.

## Key Takeaways

- IMUs do **not** provide position or velocity directly.  
- Gyroscope bias causes unbounded orientation drift.  
- Orientation errors directly corrupt gravity compensation.  
- Incorrect gravity removal creates fake linear acceleration.  
- Double integration amplifies even tiny errors catastrophically.  

**IMU-only dead reckoning is fundamentally unstable.**

This is why real systems always use IMUs as:
- High-rate **prediction**.
- Combined with other sensors for **correction** (camera, GPS, encoders).

## Why This Matters

This failure mode explains the necessity of:
- Sensor fusion (EKF, UKF, factor graphs).
- External orientation or position references.
- Careful frame and gravity handling in robotics stacks.

IMUs are not broken.  
They are **honest sensors** whose limitations must be designed around.

## How to Run

```bash
python3 imu_drift_demo.py
```

Generated plots are saved in **results/**.

---