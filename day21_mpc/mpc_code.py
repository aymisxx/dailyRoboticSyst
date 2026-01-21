"""
Day 21: Why MPC Exists (Constraints)
------------------------------------
Minimal demo: LQR vs MPC on a double-integrator with actuator limits.

System:
  x = [position, velocity]
  u = acceleration (bounded)

Key idea:
  - LQR assumes no hard constraints.
  - Real actuators saturate -> LQR becomes "optimal fantasy + clipping"
  - MPC bakes constraints into the optimization -> feasible plans, smoother behavior.

Outputs (saved to ./results):
  day21_positions.png
  day21_velocities.png
  day21_controls.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize



# Utilities

def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def dlqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray):
    """Discrete-time LQR gain K via DARE."""
    # P solves: A^T P A - P - A^T P B (B^T P B + R)^-1 B^T P A + Q = 0
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K, P


def simulate_step(A: np.ndarray, B: np.ndarray, x: np.ndarray, u: float) -> np.ndarray:
    """
    x_{k+1} = A x_k + B u_k
    Uses B.flatten() so return is always shape (2,) for this problem.
    """
    return A @ x + B.flatten() * float(u)



# MPC Solver (SLSQP)

def solve_mpc_slsqp(
    A: np.ndarray,
    B: np.ndarray,
    x0: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    Qf: np.ndarray,
    N: int,
    u_min: float,
    u_max: float,
    x_ref: np.ndarray | None = None,
):
    """
    Finite-horizon constrained MPC:
      min sum_{k=0..N-1} (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k
          + (x_N - x_ref)^T Qf (x_N - x_ref)
      s.t. u_min <= u_k <= u_max

    Decision variables: u_0..u_{N-1} (scalar each)
    """
    if x_ref is None:
        x_ref = np.zeros_like(x0)

    def rollout(u_seq: np.ndarray) -> np.ndarray:
        x = x0.copy()
        xs = [x]
        for uk in u_seq:
            x = simulate_step(A, B, x, float(uk))
            xs.append(x)
        return np.array(xs)

    def cost(u_seq: np.ndarray) -> float:
        xs = rollout(u_seq)
        J = 0.0
        for k in range(N):
            e = xs[k] - x_ref
            J += float(e.T @ Q @ e) + float(u_seq[k] ** 2) * float(R[0, 0])
        eN = xs[N] - x_ref
        J += float(eN.T @ Qf @ eN)
        return float(J)

    u0 = np.zeros(N, dtype=float)
    bounds = [(u_min, u_max)] * N

    res = minimize(
        cost,
        u0,
        method="SLSQP",
        bounds=bounds,
        options={"maxiter": 200, "ftol": 1e-7, "disp": False},
    )

    if not res.success:
        # Safe fallback: do nothing (still feasible)
        u_seq = np.clip(u0, u_min, u_max)
    else:
        u_seq = res.x

    return float(u_seq[0]), u_seq, res.success



# Main

def main():
    # Discrete double integrator
    dt = 0.1
    A = np.array([[1.0, dt],
                  [0.0, 1.0]], dtype=float)
    B = np.array([[0.5 * dt * dt],
                  [dt]], dtype=float)

    # Costs
    Q = np.diag([20.0, 2.0])
    R = np.array([[0.8]])
    Qf = np.diag([30.0, 4.0])

    # Constraints (actuator accel limit)
    u_min, u_max = -1.0, 1.0

    # Horizon
    N = 20

    # Simulation
    T = 10.0
    steps = int(T / dt)

    # Initial condition
    x0 = np.array([8.0, 0.0], dtype=float)
    x_ref = np.array([0.0, 0.0], dtype=float)

    # LQR gain
    K, _ = dlqr(A, B, Q, R)

    # Storage
    xs_lqr = np.zeros((steps + 1, 2), dtype=float)
    us_lqr = np.zeros(steps, dtype=float)

    xs_mpc = np.zeros((steps + 1, 2), dtype=float)
    us_mpc = np.zeros(steps, dtype=float)

    xs_lqr[0] = x0.copy()
    xs_mpc[0] = x0.copy()

    # Simulate
    for t in range(steps):
        # LQR (unconstrained) then saturation
        x_l = xs_lqr[t]
        u_l = float(-(K @ x_l)[0])  # unconstrained "optimal"
        u_l_sat = float(np.clip(u_l, u_min, u_max))  # reality slap
        us_lqr[t] = u_l_sat
        xs_lqr[t + 1] = simulate_step(A, B, x_l, u_l_sat)

        # MPC (constrained optimization)
        x_m = xs_mpc[t]
        u_m, _, ok = solve_mpc_slsqp(A, B, x_m, Q, R, Qf, N, u_min, u_max, x_ref=x_ref)
        # even if ok=False, u_m is still feasible due to fallback
        us_mpc[t] = u_m
        xs_mpc[t + 1] = simulate_step(A, B, x_m, u_m)

    # Results directory (works in script + notebook)
    base_dir = os.path.dirname(__file__) if "__file__" in globals() else os.getcwd()
    results_dir = os.path.join(base_dir, "results")
    ensure_dir(results_dir)

    time = np.arange(steps + 1) * dt
    time_u = np.arange(steps) * dt

    # Plot: position
    plt.figure()
    plt.plot(time, xs_lqr[:, 0], label="LQR (with saturation)")
    plt.plot(time, xs_mpc[:, 0], label="MPC (constrained)")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.title("Day 21: Position: LQR+Clip vs Constrained MPC")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day21_positions.png"), dpi=200)
    plt.close()

    # Plot: velocity
    plt.figure()
    plt.plot(time, xs_lqr[:, 1], label="LQR (with saturation)")
    plt.plot(time, xs_mpc[:, 1], label="MPC (constrained)")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("velocity")
    plt.title("Day 21: Velocity: LQR+Clip vs Constrained MPC")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day21_velocities.png"), dpi=200)
    plt.close()

    # Plot: control
    plt.figure()
    plt.plot(time_u, us_lqr, label="LQR command (clipped)")
    plt.plot(time_u, us_mpc, label="MPC command")
    plt.axhline(u_max, linestyle="--")
    plt.axhline(u_min, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("u (accel)")
    plt.title("Day 21: Control input: constraints are the whole point")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day21_controls.png"), dpi=200)
    plt.close()

    print("Day 21 MPC demo complete.")
    print(f"Saved plots to: {results_dir}")
    print(" - day21_positions.png")
    print(" - day21_velocities.png")
    print(" - day21_controls.png")
    print("\nWhat to look for:")
    print(" - LQR keeps 'wanting' more than the actuator can do, then gets clipped.")
    print(" - MPC plans within bounds, so behavior is more feasible/consistent.")


if __name__ == "__main__":
    main()