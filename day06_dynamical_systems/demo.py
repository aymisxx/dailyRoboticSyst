"""
Day 06: Ultra-minimal dynamical system demo

Discrete-time linear system:
    x_{k+1} = A x_k + B u_k
    y_k     = C x_k

State x = [position, velocity]
"""

import numpy as np

# System definition

dt = 0.1
A = np.array([
    [1.0, dt],
    [0.0, 0.9]
])

B = np.array([
    [0.0],
    [1.0]
])

C = np.array([[1.0, 0.0]])  # measure position only

# Simulation

N = 50
x = np.zeros((2, 1))

outputs = []

for k in range(N):
    u = np.array([[1.0]]) if k < 5 else np.array([[0.0]])  # short impulse
    y = C @ x

    outputs.append(y.item())
    x = A @ x + B @ u

# Results

print("Day 06 — Dynamical System Demo")
print("==============================")
print(f"Final state x_N = {x.ravel()}")
print(f"Final output y_N = {outputs[-1]:.6f}")
print()
print("First 5 outputs:", np.round(outputs[:5], 6))
print("Last 5 outputs: ", np.round(outputs[-5:], 6))