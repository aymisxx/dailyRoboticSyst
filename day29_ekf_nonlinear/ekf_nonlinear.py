import os
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)


# Nonlinear system definitions

dt = 0.1

def f(x):
    # nonlinear motion model
    return x + np.sin(x) * dt

def h(x):
    # nonlinear measurement model
    return x**2

def F_jacobian(x):
    return 1 + np.cos(x) * dt

def H_jacobian(x):
    return 2 * x


# Simulation parameters

N = 60
Q = 0.01     # process noise variance
R = 0.1      # measurement noise variance
eps = 1e-12  # numerical safety

x_true = 0.5
x_est = 0.2
P = 1.0

true_states = []
est_states = []
meas = []


# EKF loop

for _ in range(N):
    # True system + measurement
    x_true = f(x_true) + np.random.randn() * np.sqrt(Q)
    z = h(x_true) + np.random.randn() * np.sqrt(R)

    # Prediction (linearize at prior estimate)
    x_pred = f(x_est)
    F = F_jacobian(x_est)
    P_pred = F * P * F + Q

    # Update (linearize measurement at predicted state)
    H = H_jacobian(x_pred)
    S = H * P_pred * H + R
    S = max(S, eps)  # safety

    K = P_pred * H / S
    innovation = z - h(x_pred)

    x_est = x_pred + K * innovation

    # Joseph form (scalar) for stability
    P = (1 - K * H) * (1 - K * H) * P_pred + (K * K) * R

    true_states.append(x_true)
    est_states.append(x_est)
    meas.append(z)


# Plot

os.makedirs("results", exist_ok=True)

plt.figure(figsize=(8, 4))
plt.plot(true_states, label="True state")
plt.plot(est_states, label="EKF estimate", linestyle="--")
plt.xlabel("Time step")
plt.ylabel("State")
plt.title("Day 29: Extended Kalman Filter (Nonlinear System)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("results/day29_ekf_timeseries.png", dpi=300)
plt.close()