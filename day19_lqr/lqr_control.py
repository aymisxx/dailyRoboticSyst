"""
Day 19: Linear Quadratic Regulator (LQR)

- Plant: Mass-Spring-Damper (continuous-time), x=[pos, vel]
- Controllers:
  (1) Pole placement state feedback: u = -K_pp x
  (2) LQR optimal state feedback:    u = -K_lqr x
- Simulation: solve_ivp
- Outputs: plots + JSON summary in ./results/

Dependencies: numpy, scipy, matplotlib
"""

from __future__ import annotations

import os
import json
from dataclasses import dataclass
from datetime import datetime

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def control_energy(t: NDArray[np.float64], u: NDArray[np.float64]) -> float:
    """∫ u(t)^2 dt via trapezoidal rule."""
    return float(np.trapz(u**2, t))


def peak_abs(y: NDArray[np.float64]) -> float:
    return float(np.max(np.abs(y)))


def settling_time(
    t: NDArray[np.float64],
    y: NDArray[np.float64],
    rel_tol: float = 0.02,
    abs_tol: float = 1e-3,
) -> float:
    """
    Settling time for regulation to 0:
    First time after which |y| <= max(rel_tol*|y0|, abs_tol) for all remaining times.
    """
    y0 = float(abs(y[0]))
    band = max(rel_tol * y0, abs_tol)
    inside = np.abs(y) <= band

    for i in range(len(t)):
        if np.all(inside[i:]):
            return float(t[i])
    return float("nan")


@dataclass
class MSDParams:
    m: float = 1.0
    c: float = 0.8
    k: float = 4.0


def msd_state_space(p: MSDParams) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    x = [position; velocity]
    xdot = A x + B u
    u = force
    """
    A = np.array([[0.0, 1.0],
                  [-p.k / p.m, -p.c / p.m]], dtype=float)
    B = np.array([[0.0],
                  [1.0 / p.m]], dtype=float)
    return A, B


def lqr_gain(
    A: NDArray[np.float64],
    B: NDArray[np.float64],
    Q: NDArray[np.float64],
    R: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    Continuous-time LQR:
    Solve A^T P + P A - P B R^-1 B^T P + Q = 0
    Then K = R^-1 B^T P
    """
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def pole_place_gain(A: NDArray[np.float64], B: NDArray[np.float64], poles: list[complex]) -> NDArray[np.float64]:
    placed = place_poles(A, B, poles, method="YT")
    return placed.gain_matrix


def simulate_closed_loop(
    A: NDArray[np.float64],
    B: NDArray[np.float64],
    K: NDArray[np.float64],
    x0: NDArray[np.float64],
    t_span: tuple[float, float],
    dt: float,
) -> dict:
    """
    Simulate xdot = (A - B K) x, with u(t) = -K x(t)
    """
    Acl = A - B @ K

    def dyn(_t: float, x: NDArray[np.float64]) -> NDArray[np.float64]:
        return (Acl @ x.reshape(-1, 1)).ravel()

    t_eval = np.arange(t_span[0], t_span[1] + 1e-12, dt, dtype=float)
    sol = solve_ivp(
        dyn,
        t_span,
        x0.astype(float),
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
        max_step=dt,  # keeps integration behavior consistent
    )

    X = sol.y.astype(float)  # shape (n, T)
    U = np.array([-(K @ X[:, i].reshape(-1, 1)).item() for i in range(X.shape[1])], dtype=float)

    return {
        "t": sol.t.astype(float),
        "X": X,
        "U": U,
        "success": bool(sol.success),
        "message": str(sol.message),
    }


def main() -> None:
    p = MSDParams(m=1.0, c=0.8, k=4.0)
    A, B = msd_state_space(p)

    # Initial condition (regulation task)
    x0 = np.array([1.0, 0.0], dtype=float)

    # Time
    t_span = (0.0, 10.0)
    dt = 0.002

    # Pole placement
    poles = [-2.5 + 2.5j, -2.5 - 2.5j]
    K_pp = pole_place_gain(A, B, poles)

    # LQR weights
    Q = np.diag([30.0, 4.0])
    R = np.array([[1.0]], dtype=float)
    K_lqr = lqr_gain(A, B, Q, R)

    # Simulate
    sim_pp = simulate_closed_loop(A, B, K_pp, x0, t_span, dt)
    sim_lqr = simulate_closed_loop(A, B, K_lqr, x0, t_span, dt)

    if not sim_pp["success"] or not sim_lqr["success"]:
        print("Integration warning:")
        print("PP :", sim_pp["message"])
        print("LQR:", sim_lqr["message"])

    def metrics(sim: dict) -> dict:
        t = sim["t"]
        x1 = sim["X"][0, :]
        x2 = sim["X"][1, :]
        u = sim["U"]
        return {
            "peak|x1|": peak_abs(x1),
            "peak|x2|": peak_abs(x2),
            "settle_x1": settling_time(t, x1, rel_tol=0.02, abs_tol=1e-3),
            "settle_x2": settling_time(t, x2, rel_tol=0.02, abs_tol=1e-3),
            "energy_int_u2": control_energy(t, u),
            "peak|u|": peak_abs(u),
        }

    met_pp = metrics(sim_pp)
    met_lqr = metrics(sim_lqr)

    np.set_printoptions(precision=6, suppress=True)
    print("Day 19: LQR")
    print(f"MSD params: m={p.m}, c={p.c}, k={p.k}")
    print("\nK_pp (Pole Placement) =", K_pp)
    print("Target poles =", poles)
    print("\nK_lqr (LQR) =", K_lqr)
    print("Q =", Q)
    print("R =", R)

    print("\nMetrics (Regulation to 0 from x0=[1,0])")
    print("Pole Placement:", met_pp)
    print("LQR:", met_lqr)

    # Save results
    results_dir = os.path.join(os.getcwd(), "results")
    ensure_dir(results_dir)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    t_pp, X_pp, U_pp = sim_pp["t"], sim_pp["X"], sim_pp["U"]
    t_lq, X_lq, U_lq = sim_lqr["t"], sim_lqr["X"], sim_lqr["U"]

    # Figure 1: states (two subplots for clarity)
    fig1 = plt.figure(figsize=(10, 6))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(t_pp, X_pp[0, :], label="pos (PP)")
    ax1.plot(t_lq, X_lq[0, :], label="pos (LQR)")
    ax1.set_ylabel("Position")
    ax1.grid(True, alpha=0.25)
    ax1.legend()

    ax2 = plt.subplot(2, 1, 2)
    ax2.plot(t_pp, X_pp[1, :], label="vel (PP)")
    ax2.plot(t_lq, X_lq[1, :], label="vel (LQR)")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity")
    ax2.grid(True, alpha=0.25)
    ax2.legend()

    fig1.suptitle("States: Pole Placement vs LQR")
    fig1_path = os.path.join(results_dir, f"day19_states_{stamp}.png")
    plt.tight_layout()
    plt.savefig(fig1_path, dpi=200)

    # Figure 2: control
    fig2 = plt.figure(figsize=(10, 4.5))
    plt.plot(t_pp, U_pp, label="u (PP)")
    plt.plot(t_lq, U_lq, label="u (LQR)")
    plt.xlabel("Time [s]")
    plt.ylabel("Control u")
    plt.title("Control Effort: Pole Placement vs LQR")
    plt.grid(True, alpha=0.25)
    plt.legend()
    fig2_path = os.path.join(results_dir, f"day19_control_{stamp}.png")
    plt.tight_layout()
    plt.savefig(fig2_path, dpi=200)

    out = {
        "params": p.__dict__,
        "x0": x0.tolist(),
        "t_span": list(t_span),
        "dt": dt,
        "poles": [str(p_) for p_ in poles],
        "K_pp": K_pp.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "K_lqr": K_lqr.tolist(),
        "metrics_pp": met_pp,
        "metrics_lqr": met_lqr,
        "fig_states": os.path.basename(fig1_path),
        "fig_control": os.path.basename(fig2_path),
    }

    json_path = os.path.join(results_dir, f"day19_summary_{stamp}.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    print("\nSaved:")
    print(f"- {fig1_path}")
    print(f"- {fig2_path}")
    print(f"- {json_path}")

    plt.show()


if __name__ == "__main__":
    main()