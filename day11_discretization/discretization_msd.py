"""
Day 11: Discretization (CT -> DT)
---------------------------------
System: Mass-Spring-Damper (MSD)

Continuous-time:
  x_dot = A x + B u

Discrete-time:
  x_{k+1} = A_d x_k + B_d u_k

We compare:
1) Exact discretization (ZOH) using matrix exponential
2) Forward Euler discretization (approx; can destabilize)

Outputs (saved in results/):
- trajectories_position_ct_vs_dt.png
- trajectories_velocity_ct_vs_dt.png
- discrete_eigs_unit_circle.png
"""

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt

try:
    from scipy.linalg import expm
except ImportError as e:
    raise ImportError(
        "scipy is required for exact discretization (expm). "
        "Install with: pip install scipy"
    ) from e


def msd_ct_matrices(m: float, c: float, k: float) -> tuple[np.ndarray, np.ndarray]:
    """
    State: x = [position, velocity]^T
    Input: u = force
    """
    A = np.array([[0.0, 1.0],
                  [-k / m, -c / m]], dtype=float)
    B = np.array([[0.0],
                  [1.0 / m]], dtype=float)
    return A, B


def discretize_zoh(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Exact ZOH discretization via augmented matrix exponential:

      M = [A B;
           0 0]

      expm(M*dt) = [Ad Bd;
                    0  I]

    Extract Ad, Bd.
    """
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m), dtype=float)
    M[:n, :n] = A
    M[:n, n:] = B

    Md = expm(M * dt)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:]
    return Ad, Bd


def discretize_forward_euler(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Forward Euler:
      x_{k+1} = x_k + dt*(A x_k + B u_k)
             = (I + dt*A)x_k + (dt*B)u_k
    """
    n = A.shape[0]
    Ad = np.eye(n) + dt * A
    Bd = dt * B
    return Ad, Bd


def simulate_dt(Ad: np.ndarray, Bd: np.ndarray, x0: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    DT simulation:
      x[k+1] = Ad x[k] + Bd u[k]

    u: shape (N, nu)
    returns X: shape (N+1, n)
    """
    N = u.shape[0]
    n = x0.shape[0]
    X = np.zeros((N + 1, n), dtype=float)
    X[0] = x0

    for k in range(N):
        X[k + 1] = (Ad @ X[k].reshape(-1, 1) + Bd @ u[k].reshape(-1, 1)).ravel()

    return X


def simulate_ct_rk4(A: np.ndarray, B: np.ndarray, x0: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    CT simulation using RK4 with ZOH input on each interval [k*dt, (k+1)*dt).

    u: shape (N, nu), held constant within each step
    returns X: shape (N+1, n)
    """
    def f(x: np.ndarray, uk: np.ndarray) -> np.ndarray:
        return (A @ x.reshape(-1, 1) + B @ uk.reshape(-1, 1)).ravel()

    N = u.shape[0]
    n = x0.shape[0]
    X = np.zeros((N + 1, n), dtype=float)
    X[0] = x0
    x = x0.copy()

    for k in range(N):
        uk = u[k].ravel()
        k1 = f(x, uk)
        k2 = f(x + 0.5 * dt * k1, uk)
        k3 = f(x + 0.5 * dt * k2, uk)
        k4 = f(x + dt * k3, uk)
        x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        X[k + 1] = x

    return X


def plot_unit_circle(ax: plt.Axes) -> None:
    th = np.linspace(0, 2*np.pi, 500)
    ax.plot(np.cos(th), np.sin(th))
    ax.axhline(0.0, linewidth=0.8)
    ax.axvline(0.0, linewidth=0.8)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Re")
    ax.set_ylabel("Im")
    ax.set_title("Discrete-time eigenvalues vs unit circle")


def main() -> None:

    # Parameters (same vibe as your earlier MSD days)

    m, c, k = 1.0, 0.4, 4.0

    dt = 0.02
    t_end_desired = 10.0

    # Make horizon consistent with dt (important!)
    N = int(np.round(t_end_desired / dt))
    t_end = N * dt
    t = np.arange(N + 1) * dt

    # Input: step force (ZOH)
    u_step = 1.0
    u = np.ones((N, 1), dtype=float) * u_step

    # Initial state: [position, velocity]
    x0 = np.array([1.0, 0.0], dtype=float)

    A, B = msd_ct_matrices(m, c, k)


    # Simulations

    X_ct = simulate_ct_rk4(A, B, x0, u, dt)                 # CT reference
    Ad_zoh, Bd_zoh = discretize_zoh(A, B, dt)
    X_zoh = simulate_dt(Ad_zoh, Bd_zoh, x0, u)              # exact DT
    Ad_eu, Bd_eu = discretize_forward_euler(A, B, dt)
    X_eu = simulate_dt(Ad_eu, Bd_eu, x0, u)                 # Euler DT


    # Eigenvalues

    eig_ct = np.linalg.eigvals(A)
    eig_zoh = np.linalg.eigvals(Ad_zoh)
    eig_eu = np.linalg.eigvals(Ad_eu)


    # Console summary (day log friendly)

    print("Day 11: Discretization (MSD)")
    print("----------------------------")
    print(f"m={m}, c={c}, k={k}")
    print(f"dt={dt}, N={N}, sim_horizon={t_end:.6f} s")
    print("\nA (CT):\n", A)
    print("B (CT):\n", B)
    print("\nCT eigenvalues (A):")
    for lam in eig_ct:
        print(f"  {lam.real:+.6f} {lam.imag:+.6f}j")

    print("\nDT eigenvalues (Ad) — ZOH:")
    for lam in eig_zoh:
        print(f"  {lam.real:+.6f} {lam.imag:+.6f}j")

    print("\nDT eigenvalues (Ad) — Euler:")
    for lam in eig_eu:
        print(f"  {lam.real:+.6f} {lam.imag:+.6f}j")


    # Save plots

    out_dir = "results"
    os.makedirs(out_dir, exist_ok=True)

    # Position
    fig1 = plt.figure()
    plt.plot(t, X_ct[:, 0], label="CT (RK4) position")
    plt.plot(t, X_zoh[:, 0], label="DT ZOH position")
    plt.plot(t, X_eu[:, 0], label="DT Euler position")
    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.title("Position: CT vs DT (ZOH vs Euler)")
    plt.grid(True)
    plt.legend()
    fig1.savefig(os.path.join(out_dir, "trajectories_position_ct_vs_dt.png"),
                 dpi=200, bbox_inches="tight")
    plt.close(fig1)

    # Velocity
    fig2 = plt.figure()
    plt.plot(t, X_ct[:, 1], label="CT (RK4) velocity")
    plt.plot(t, X_zoh[:, 1], label="DT ZOH velocity")
    plt.plot(t, X_eu[:, 1], label="DT Euler velocity")
    plt.xlabel("time [s]")
    plt.ylabel("velocity")
    plt.title("Velocity: CT vs DT (ZOH vs Euler)")
    plt.grid(True)
    plt.legend()
    fig2.savefig(os.path.join(out_dir, "trajectories_velocity_ct_vs_dt.png"),
                 dpi=200, bbox_inches="tight")
    plt.close(fig2)

    # Discrete eigenvalues on unit circle
    fig3, ax3 = plt.subplots()
    plot_unit_circle(ax3)
    ax3.scatter(eig_zoh.real, eig_zoh.imag, label="ZOH Ad eigs")
    ax3.scatter(eig_eu.real, eig_eu.imag, label="Euler Ad eigs")
    ax3.grid(True)
    ax3.legend()
    fig3.savefig(os.path.join(out_dir, "discrete_eigs_unit_circle.png"),
                 dpi=200, bbox_inches="tight")
    plt.close(fig3)

    print(f"\nSaved plots in: {out_dir}/")
    print(" - trajectories_position_ct_vs_dt.png")
    print(" - trajectories_velocity_ct_vs_dt.png")
    print(" - discrete_eigs_unit_circle.png")


if __name__ == "__main__":
    main()