"""
Day 17: State Feedback (Full-State Feedback)

This script demonstrates basic continuous-time state feedback:
    x_{dot} = A x + B u
    u     = -K x

We compare open-loop vs closed-loop trajectories and save plots to ./results.

Dependencies:
    numpy, scipy, matplotlib

Run:
    python state_feedback.py
"""

from __future__ import annotations

import os
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import place_poles
from scipy.integrate import solve_ivp


@dataclass
class MSDParams:
    """Mass-spring-damper parameters."""
    m: float = 1.0   # kg
    c: float = 0.5   # N*s/m
    k: float = 2.0   # N/m


def msd_state_space(p: MSDParams) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Mass-spring-damper with force input u.

    State: x = [position; velocity]
    Dynamics:
        x1_dot = x2
        x2_dot = -(k/m)*x1 -(c/m)*x2 + (1/m)*u
    Output: y = position
    """
    A = np.array([[0.0, 1.0],
                  [-(p.k / p.m), -(p.c / p.m)]], dtype=float)
    B = np.array([[0.0],
                  [1.0 / p.m]], dtype=float)
    C = np.array([[1.0, 0.0]], dtype=float)
    return A, B, C


def sat(u: float, umax: float | None) -> float:
    """Optional actuator saturation."""
    if umax is None:
        return u
    return float(np.clip(u, -umax, umax))


def simulate_open_loop(A: np.ndarray, B: np.ndarray, x0: np.ndarray,
                       t_final: float, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Open-loop simulation with u(t)=0."""
    def f(t, x):
        u = 0.0
        return (A @ x.reshape(-1, 1) + B * u).ravel()

    t_eval = np.arange(0.0, t_final + 1e-12, dt)
    sol = solve_ivp(f, (0.0, t_final), x0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    U = np.zeros_like(sol.t)
    return sol.t, sol.y, U


def simulate_state_feedback(A: np.ndarray, B: np.ndarray, K: np.ndarray, x0: np.ndarray,
                           t_final: float, dt: float, umax: float | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Closed-loop simulation with u(t) = -K x(t), optional saturation."""
    K = np.asarray(K, dtype=float).reshape(1, -1)

    def f(t, x):
        u = -float(K @ x.reshape(-1, 1))
        u = sat(u, umax)
        return (A @ x.reshape(-1, 1) + B * u).ravel()

    t_eval = np.arange(0.0, t_final + 1e-12, dt)
    sol = solve_ivp(f, (0.0, t_final), x0, t_eval=t_eval, rtol=1e-8, atol=1e-10)

    # Recompute u(t) on the solution grid for plotting
    U = np.array([-sat(float(K @ sol.y[:, i].reshape(-1, 1)), umax) for i in range(sol.y.shape[1])], dtype=float)
    return sol.t, sol.y, U


def main() -> None:

    # 1) Build a simple system

    p = MSDParams(m=1.0, c=0.5, k=2.0)
    A, B, C = msd_state_space(p)


    # 2) Choose desired poles

    # Tip: more negative => faster, but usually higher control effort.
    desired_poles = np.array([-2.0 + 2.0j, -2.0 - 2.0j])


    # 3) Compute K via pole placement

    placed = place_poles(A, B, desired_poles)
    K = placed.gain_matrix  # shape (1, n)


    # 4) Simulate

    x0 = np.array([1.0, 0.0], dtype=float)  # initial position 1 m, zero velocity
    t_final = 8.0
    dt = 0.01

    # Optional saturation (set to None to disable)
    umax = 10.0  # N

    t_ol, X_ol, U_ol = simulate_open_loop(A, B, x0, t_final, dt)
    t_cl, X_cl, U_cl = simulate_state_feedback(A, B, K, x0, t_final, dt, umax=umax)


    # 5) Save plots

    out_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(out_dir, exist_ok=True)

    # States: position + velocity
    plt.figure()
    plt.plot(t_ol, X_ol[0, :], label="pos open-loop")
    plt.plot(t_cl, X_cl[0, :], label="pos closed-loop")
    plt.xlabel("t [s]")
    plt.ylabel("position [m]")
    plt.title("State Feedback: Position (Open vs Closed)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "position_open_vs_closed.png"), dpi=200)

    plt.figure()
    plt.plot(t_ol, X_ol[1, :], label="vel open-loop")
    plt.plot(t_cl, X_cl[1, :], label="vel closed-loop")
    plt.xlabel("t [s]")
    plt.ylabel("velocity [m/s]")
    plt.title("State Feedback: Velocity (Open vs Closed)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "velocity_open_vs_closed.png"), dpi=200)

    # Control input
    plt.figure()
    plt.plot(t_ol, U_ol, label="u open-loop (0)")
    plt.plot(t_cl, U_cl, label="u closed-loop = -Kx (sat)")
    plt.xlabel("t [s]")
    plt.ylabel("u [N]")
    plt.title("Control Effort")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "control_input.png"), dpi=200)

    # Console summary
    eig_ol = np.linalg.eigvals(A)
    eig_cl = np.linalg.eigvals(A - B @ K)
    print("Day 17: State Feedback")
    print(f"MSD params: m={p.m}, c={p.c}, k={p.k}")
    print("Open-loop eigenvalues:", eig_ol)
    print("Desired poles:", desired_poles)
    print("K gain matrix:", K)
    print("Closed-loop eigenvalues:", eig_cl)
    if umax is not None:
        print(f"Actuator saturation: |u| <= {umax} N")
    print(f"Saved plots to: {out_dir}")


if __name__ == "__main__":
    main()