#!/usr/bin/env python3
"""
Day 27: Kalman Filter (Vector)

Minimal vector Kalman filter example:
- State: x = [position, velocity]^T
- Dynamics: constant-velocity model
- Measurement: position only
- Runs simulation with process noise + measurement noise
- Produces plots and saves to ./results/

Run:
    python kalman_vector.py

Outputs:
    results/day27_kf_vector_timeseries.png
    results/day27_kf_vector_cov_trace.png
"""

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def kalman_filter_vector_demo(
    *,
    dt: float = 0.1,
    steps: int = 300,
    x0_true: np.ndarray | None = None,
    x0_est: np.ndarray | None = None,
    P0: np.ndarray | None = None,
    q_accel: float = 0.5,
    r_pos: float = 2.0,
    seed: int = 7,
) -> dict:
    """
    Constant-velocity vector Kalman filter demo.

    Process noise model:
      - We assume unknown acceleration as process noise.
      - Q derived from continuous-time acceleration noise (variance q_accel^2).

    Measurement noise:
      z = H x + v, v ~ N(0, R), where H observes position only.
    """
    rng = np.random.default_rng(seed)

    # State transition (constant velocity)
    F = np.array([[1.0, dt],
                  [0.0, 1.0]], dtype=float)

    # Measurement model: observe position only
    H = np.array([[1.0, 0.0]], dtype=float)

    # Process noise covariance Q using acceleration noise
    # Standard discrete-time CV model:
    # Q = sigma_a^2 * [[dt^4/4, dt^3/2],
    #                  [dt^3/2, dt^2   ]]
    sigma_a2 = float(q_accel) ** 2
    Q = sigma_a2 * np.array([[dt**4 / 4.0, dt**3 / 2.0],
                             [dt**3 / 2.0, dt**2]], dtype=float)

    # Measurement noise covariance
    R = np.array([[float(r_pos) ** 2]], dtype=float)

    # Initial conditions
    if x0_true is None:
        x_true = np.array([0.0, 1.0], dtype=float)  # start at 0m, 1 m/s
    else:
        x_true = np.array(x0_true, dtype=float).reshape(2)

    if x0_est is None:
        x_est = np.array([0.0, 0.0], dtype=float)  # wrong-ish guess on velocity
    else:
        x_est = np.array(x0_est, dtype=float).reshape(2)

    if P0 is None:
        P = np.diag([10.0**2, 5.0**2]).astype(float)  # big initial uncertainty
    else:
        P = np.array(P0, dtype=float).reshape(2, 2)

    # Storage
    t = np.arange(steps) * dt
    xs_true = np.zeros((steps, 2))
    xs_est = np.zeros((steps, 2))
    zs = np.zeros((steps, 1))
    Ps = np.zeros((steps, 2, 2))
    Ks = np.zeros((steps, 2, 1))
    innov = np.zeros((steps, 1))

    # Helper: sample process noise ~ N(0, Q)
    # We'll do it via Cholesky for stability.
    Lq = np.linalg.cholesky(Q + 1e-12 * np.eye(2))
    Lr = np.linalg.cholesky(R + 1e-12 * np.eye(1))

    for k in range(steps):
        # Simulate true system
        w = Lq @ rng.standard_normal(2)  # process noise
        x_true = F @ x_true + w

        # Measurement
        v = Lr @ rng.standard_normal(1)
        z = H @ x_true + v

        # Kalman predict
        x_pred = F @ x_est
        P_pred = F @ P @ F.T + Q

        # Kalman update
        y = z - (H @ x_pred)                 # innovation
        S = H @ P_pred @ H.T + R             # innovation covariance
        K = P_pred @ H.T @ np.linalg.inv(S)  # Kalman gain

        x_est = x_pred + (K @ y).reshape(2)
        P = (np.eye(2) - K @ H) @ P_pred

        # Store
        xs_true[k] = x_true
        xs_est[k] = x_est
        zs[k] = z
        Ps[k] = P
        Ks[k] = K
        innov[k] = y

    return {
        "t": t,
        "xs_true": xs_true,
        "xs_est": xs_est,
        "zs": zs,
        "Ps": Ps,
        "Ks": Ks,
        "innov": innov,
        "F": F,
        "H": H,
        "Q": Q,
        "R": R,
    }


def main() -> None:
    results_dir = "results"
    ensure_dir(results_dir)

    data = kalman_filter_vector_demo(
        dt=0.1,
        steps=350,
        q_accel=0.8,
        r_pos=2.5,
        seed=11,
    )

    t = data["t"]
    xs_true = data["xs_true"]
    xs_est = data["xs_est"]
    zs = data["zs"].reshape(-1)
    Ps = data["Ps"]

    # Plot 1: timeseries (position + velocity)
    fig1 = plt.figure(figsize=(10, 7))

    ax1 = fig1.add_subplot(2, 1, 1)
    ax1.plot(t, xs_true[:, 0], label="True position")
    ax1.scatter(t, zs, s=10, label="Measured position (noisy)", alpha=0.6)
    ax1.plot(t, xs_est[:, 0], label="KF estimated position")
    ax1.set_ylabel("Position")
    ax1.grid(True)
    ax1.legend(loc="best")

    ax2 = fig1.add_subplot(2, 1, 2)
    ax2.plot(t, xs_true[:, 1], label="True velocity")
    ax2.plot(t, xs_est[:, 1], label="KF estimated velocity")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity")
    ax2.grid(True)
    ax2.legend(loc="best")

    out1 = os.path.join(results_dir, "day27_kf_vector_timeseries.png")
    fig1.tight_layout()
    fig1.savefig(out1, dpi=200)
    plt.close(fig1)

    # Plot 2: covariance trace + diagonals (how uncertainty shrinks/moves)
    Pxx = Ps[:, 0, 0]
    Pvv = Ps[:, 1, 1]
    Ptr = Ps[:, 0, 0] + Ps[:, 1, 1]

    fig2 = plt.figure(figsize=(10, 5))
    ax = fig2.add_subplot(1, 1, 1)
    ax.plot(t, Pxx, label="Var(position) = P[0,0]")
    ax.plot(t, Pvv, label="Var(velocity) = P[1,1]")
    ax.plot(t, Ptr, label="trace(P)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Covariance values")
    ax.grid(True)
    ax.legend(loc="best")

    out2 = os.path.join(results_dir, "day27_kf_vector_cov_trace.png")
    fig2.tight_layout()
    fig2.savefig(out2, dpi=200)
    plt.close(fig2)

    print("Day 27 vector KF done.")
    print(f"Saved:\n- {out1}\n- {out2}")
    print("\nTip: tweak q_accel (process noise) and r_pos (measurement noise) "
          "to see the filter trust the model vs sensor more.")


if __name__ == "__main__":
    main()