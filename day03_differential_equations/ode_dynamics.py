# Day 03: Differential Equations
# Numerical simulation of 1st and 2nd order ODEs
#
# Run:
#   python ode_dynamics.py
#
# Outputs:
#   results/first_order_response.png
#   results/second_order_response.png

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt


def ensure_results_dir(path: str = "results") -> str:
    os.makedirs(path, exist_ok=True)
    return path


# First Order ODE
# dx/dt = -a x
# Explicit Euler

def simulate_first_order(a: float = 1.0, x0: float = 1.0, t_end: float = 10.0, dt: float = 0.01):
    if dt <= 0:
        raise ValueError("dt must be > 0")

    t = np.arange(0.0, t_end + dt, dt)  # include endpoint
    x = np.zeros_like(t)
    x[0] = x0

    for i in range(1, len(t)):
        x[i] = x[i - 1] + dt * (-a * x[i - 1])

    return t, x



# Second Order ODE
# x'' + 2ζω_n x' + ω_n^2 x = 0
# Convert to:
#   x' = v
#   v' = -2ζω_n v - ω_n^2 x
#
# Semi-implicit Euler:
#   v_{k+1} = v_k + dt * a_k
#   x_{k+1} = x_k + dt * v_{k+1}

def simulate_second_order(
    wn: float = 2.0,
    zeta: float = 0.3,
    x0: float = 1.0,
    v0: float = 0.0,
    t_end: float = 10.0,
    dt: float = 0.01,
):
    if dt <= 0:
        raise ValueError("dt must be > 0")
    if wn <= 0:
        raise ValueError("wn must be > 0")
    if zeta < 0:
        raise ValueError("zeta must be >= 0")

    t = np.arange(0.0, t_end + dt, dt)  # include endpoint
    x = np.zeros_like(t)
    v = np.zeros_like(t)

    x[0] = x0
    v[0] = v0

    for i in range(1, len(t)):
        a = -2.0 * zeta * wn * v[i - 1] - (wn ** 2) * x[i - 1]
        v[i] = v[i - 1] + dt * a
        x[i] = x[i - 1] + dt * v[i]  # semi-implicit update

    return t, x, v


def main():
    results_dir = ensure_results_dir()

    # First-order
    t1, x1 = simulate_first_order(a=1.0, x0=1.0, t_end=10.0, dt=0.01)
    plt.figure()
    plt.plot(t1, x1)
    plt.title("First-Order ODE Response: dx/dt = -a x")
    plt.xlabel("Time (s)")
    plt.ylabel("x(t)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "first_order_response.png"), dpi=200)
    plt.close()

    # Second-order
    t2, x2, v2 = simulate_second_order(wn=2.0, zeta=0.3, x0=1.0, v0=0.0, t_end=10.0, dt=0.001)
    plt.figure()
    plt.plot(t2, x2)
    plt.title("Second-Order ODE Response: x'' + 2ζωₙ x' + ωₙ² x = 0")
    plt.xlabel("Time (s)")
    plt.ylabel("x(t)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "second_order_response.png"), dpi=200)
    plt.close()


if __name__ == "__main__":
    main()