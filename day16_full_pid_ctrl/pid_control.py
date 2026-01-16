"""
Day 16: Full PID Control
------------------------
System: 2nd-order mass–spring–damper
Controller: Full PID (P + I + D)

Outputs:
- results/pid_output_tracking.png
- results/pid_control_effort.png
- results/pid_tracking_error.png
"""

import numpy as np
import matplotlib.pyplot as plt
import os


# Create results directory

RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)


# System parameters

m = 1.0
k = 2.0
b = 0.8

A = np.array([[0, 1],
              [-k/m, -b/m]])
B = np.array([[0],
              [1/m]])
C = np.array([1, 0])


# Simulation parameters

dt = 0.01
t_final = 10.0
time = np.arange(0, t_final, dt)

r = 1.0  # reference


# PID gains

Kp = 20.0
Ki = 5.0
Kd = 6.0


# Initial conditions

x = np.array([[0.0],
              [0.0]])

integral_error = 0.0
prev_error = 0.0

x_log, u_log, e_log = [], [], []


# Simulation loop

for _ in time:
    y = float(C @ x)
    error = r - y

    integral_error += error * dt
    derivative_error = (error - prev_error) / dt

    u = Kp * error + Ki * integral_error + Kd * derivative_error

    x_dot = A @ x + B * u
    x = x + x_dot * dt

    x_log.append(y)
    u_log.append(u)
    e_log.append(error)

    prev_error = error


# Plot 1: Output tracking

plt.figure()
plt.plot(time, x_log, label="Output (position)")
plt.plot(time, r * np.ones_like(time), "--", label="Reference")
plt.xlabel("Time [s]")
plt.ylabel("Position")
plt.title("PID Control: Output Tracking")
plt.legend()
plt.grid(True)
plt.savefig(f"{RESULTS_DIR}/pid_output_tracking.png", dpi=300)
plt.close()


# Plot 2: Control effort

plt.figure()
plt.plot(time, u_log)
plt.xlabel("Time [s]")
plt.ylabel("Control Input")
plt.title("PID Control Effort")
plt.grid(True)
plt.savefig(f"{RESULTS_DIR}/pid_control_effort.png", dpi=300)
plt.close()


# Plot 3: Tracking error

plt.figure()
plt.plot(time, e_log)
plt.xlabel("Time [s]")
plt.ylabel("Tracking Error")
plt.title("PID Tracking Error")
plt.grid(True)
plt.savefig(f"{RESULTS_DIR}/pid_tracking_error.png", dpi=300)
plt.close()

print("Day 16 complete. Results saved to /results")