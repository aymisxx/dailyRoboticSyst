"""
Day 30: When EKF Fails + Estimation vs Control (systems demo)

Simulates:
- Unicycle robot motion (nonlinear)
- Range + bearing measurements to a fixed landmark (nonlinear)
Runs two EKFs:
  1) EKF_OK: reasonable Q/R
  2) EKF_OVERCONF: too-small Q/R -> covariance collapses -> divergence risk

Also runs two control styles:
  - mild (smooth turns)
  - aggressive (hard turns)

Outputs saved to ./results:
- day30_trajectories_mild.png
- day30_errors_mild.png
- day30_innovation_nis_mild.png
- day30_covtrace_mild.png
- day30_trajectories_aggressive.png
- day30_errors_aggressive.png
- day30_innovation_nis_aggressive.png
- day30_covtrace_aggressive.png

Run:
python day30_when_ekf_fails_estimation_vs_control.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt



# Helpers

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def symmetrize(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)



# Models

def unicycle_step(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    x = [px, py, theta]
    u = [v, w]
    Euler discretization
    """
    px, py, th = x
    v, w = u

    px2 = px + v * np.cos(th) * dt
    py2 = py + v * np.sin(th) * dt
    th2 = wrap_angle(th + w * dt)

    return np.array([px2, py2, th2], dtype=float)


def motion_jacobian_F(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """F = df/dx for Euler unicycle."""
    _, _, th = x
    v, _ = u
    F = np.eye(3, dtype=float)
    F[0, 2] = -v * np.sin(th) * dt
    F[1, 2] =  v * np.cos(th) * dt
    return F


def meas_model_z(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """z = [range, bearing] to landmark; bearing in robot frame."""
    px, py, th = x
    lx, ly = landmark

    dx = lx - px
    dy = ly - py

    r = np.sqrt(dx * dx + dy * dy)
    b = wrap_angle(np.arctan2(dy, dx) - th)

    return np.array([r, b], dtype=float)


def meas_jacobian_H(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """H = dh/dx for range-bearing."""
    px, py, th = x
    lx, ly = landmark

    dx = lx - px
    dy = ly - py

    q = dx * dx + dy * dy
    q = max(q, 1e-12)
    r = np.sqrt(q)

    # range partials
    dr_dpx = -dx / r
    dr_dpy = -dy / r
    dr_dth = 0.0

    # bearing partials
    db_dpx =  dy / q
    db_dpy = -dx / q
    db_dth = -1.0

    H = np.array([
        [dr_dpx, dr_dpy, dr_dth],
        [db_dpx, db_dpy, db_dth]
    ], dtype=float)
    return H



# EKF

def ekf_predict(x: np.ndarray, P: np.ndarray, u: np.ndarray, Q: np.ndarray, dt: float):
    x_pred = unicycle_step(x, u, dt)
    F = motion_jacobian_F(x, u, dt)
    P_pred = F @ P @ F.T + Q
    return x_pred, symmetrize(P_pred)


def ekf_update(x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray, landmark: np.ndarray):
    """
    Standard EKF update with:
    - wrapped bearing innovation
    - Joseph form covariance update
    - solve() instead of inverse
    Returns: x, P, innovation y, innovation covariance S, NIS
    """
    z_hat = meas_model_z(x, landmark)
    y = z - z_hat
    y[1] = wrap_angle(y[1])

    H = meas_jacobian_H(x, landmark)
    S = H @ P @ H.T + R
    S = symmetrize(S)

    # Compute K = P H^T S^{-1} using solve for stability:
    # Solve S * X = (H P)^T  => X = S^{-1} (H P)^T
    HP = H @ P
    try:
        X = np.linalg.solve(S, HP)          # X = S^{-1} (H P)
    except np.linalg.LinAlgError:
        # jitter if S ill-conditioned
        X = np.linalg.solve(S + 1e-9*np.eye(2), HP)

    K = X.T                                # K = P H^T S^{-1}

    x_upd = x + K @ y
    x_upd[2] = wrap_angle(x_upd[2])

    I = np.eye(3)
    P_upd = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T
    P_upd = symmetrize(P_upd)

    # NIS = y^T S^{-1} y
    try:
        nis = float(y.T @ np.linalg.solve(S, y))
    except np.linalg.LinAlgError:
        nis = float(y.T @ np.linalg.solve(S + 1e-9*np.eye(2), y))

    return x_upd, P_upd, y, S, nis



# Scenario runner

def run_scenario(name: str, aggressive: bool, seed: int = 7):
    np.random.seed(seed)

    dt = 0.1
    T = 35.0
    N = int(T / dt)

    landmark = np.array([8.0, 6.0], dtype=float)

    # True initial state
    x_true = np.array([0.0, 0.0, np.deg2rad(25.0)], dtype=float)

    # Initial estimate intentionally biased (stress EKF linearization)
    x0_est = x_true + np.array([0.8, -0.6, np.deg2rad(18.0)], dtype=float)
    x0_est[2] = wrap_angle(x0_est[2])

    P0 = np.diag([1.0, 1.0, np.deg2rad(25.0) ** 2])

    # "True" noise (used to generate data)
    # Control noise: how much the executed v,w differs from commanded
    sigma_v_true = 0.03
    sigma_w_true = np.deg2rad(2.0)

    # Measurement noise
    R_true = np.diag([0.12**2, np.deg2rad(2.0) ** 2])

    # EKF tuning
    # EKF OK
    Q_ok = np.diag([0.03**2, 0.03**2, np.deg2rad(2.0) ** 2])
    R_ok = np.diag([0.14**2, np.deg2rad(2.5) ** 2])

    # EKF Overconfident (classic failure driver)
    Q_bad = np.diag([0.002**2, 0.002**2, np.deg2rad(0.2) ** 2])
    R_bad = np.diag([0.03**2, np.deg2rad(0.6) ** 2])

    # logs
    xs_true = np.zeros((N, 3))
    xs_ok = np.zeros((N, 3))
    xs_bad = np.zeros((N, 3))

    err_ok = np.zeros((N, 3))
    err_bad = np.zeros((N, 3))

    innov_ok = np.zeros((N, 2))
    innov_bad = np.zeros((N, 2))

    nis_ok = np.zeros(N)
    nis_bad = np.zeros(N)

    covtrace_ok = np.zeros(N)
    covtrace_bad = np.zeros(N)

    # init filters
    x_ok, P_ok = x0_est.copy(), P0.copy()
    x_bad, P_bad = x0_est.copy(), P0.copy()

    for k in range(N):
        t = k * dt

        # commanded control
        v_cmd = 0.35 + 0.06 * np.sin(0.5 * t)
        if aggressive:
            w_cmd = 0.9 * np.sin(1.3 * t) + 0.35 * np.sign(np.sin(0.25 * t))
        else:
            w_cmd = 0.45 * np.sin(0.9 * t)

        u_cmd = np.array([v_cmd, w_cmd], dtype=float)

        # true executed control (control noise)
        v_exec = v_cmd + np.random.randn() * sigma_v_true
        w_exec = w_cmd + np.random.randn() * sigma_w_true
        u_exec = np.array([v_exec, w_exec], dtype=float)

        # true motion
        x_true = unicycle_step(x_true, u_exec, dt)

        # measurement
        z = meas_model_z(x_true, landmark)
        z = z + np.random.multivariate_normal(mean=np.zeros(2), cov=R_true)
        z[1] = wrap_angle(z[1])

        # EKF OK: uses commanded control
        x_ok, P_ok = ekf_predict(x_ok, P_ok, u_cmd, Q_ok, dt)
        x_ok, P_ok, y_ok, _, nisA = ekf_update(x_ok, P_ok, z, R_ok, landmark)

        # EKF BAD
        x_bad, P_bad = ekf_predict(x_bad, P_bad, u_cmd, Q_bad, dt)
        x_bad, P_bad, y_bad, _, nisB = ekf_update(x_bad, P_bad, z, R_bad, landmark)

        # log
        xs_true[k] = x_true
        xs_ok[k] = x_ok
        xs_bad[k] = x_bad

        e_ok = x_ok - x_true
        e_bad = x_bad - x_true
        e_ok[2] = wrap_angle(e_ok[2])
        e_bad[2] = wrap_angle(e_bad[2])

        err_ok[k] = e_ok
        err_bad[k] = e_bad

        innov_ok[k] = y_ok
        innov_bad[k] = y_bad

        nis_ok[k] = nisA
        nis_bad[k] = nisB

        covtrace_ok[k] = float(np.trace(P_ok))
        covtrace_bad[k] = float(np.trace(P_bad))


    # Plots

    os.makedirs("results", exist_ok=True)
    tt = np.arange(N) * dt

    # Trajectories
    plt.figure(figsize=(9, 7))
    plt.plot(xs_true[:, 0], xs_true[:, 1], label="True")
    plt.plot(xs_ok[:, 0], xs_ok[:, 1], label="EKF OK")
    plt.plot(xs_bad[:, 0], xs_bad[:, 1], label="EKF Overconfident")
    plt.scatter([landmark[0]], [landmark[1]], marker="*", s=200, label="Landmark")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Day 30: Trajectories ({name})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/day30_trajectories_{name}.png", dpi=200)
    plt.close()

    # Errors
    plt.figure(figsize=(10, 7))
    plt.plot(tt, err_ok[:, 0], label="ex (OK)")
    plt.plot(tt, err_ok[:, 1], label="ey (OK)")
    plt.plot(tt, np.rad2deg(err_ok[:, 2]), label="eth deg (OK)")

    plt.plot(tt, err_bad[:, 0], label="ex (Overconf)")
    plt.plot(tt, err_bad[:, 1], label="ey (Overconf)")
    plt.plot(tt, np.rad2deg(err_bad[:, 2]), label="eth deg (Overconf)")

    plt.grid(True, alpha=0.3)
    plt.xlabel("time [s]")
    plt.ylabel("error")
    plt.title(f"Day 30: State Estimation Error ({name})")
    plt.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(f"results/day30_errors_{name}.png", dpi=200)
    plt.close()

    # Innovation + NIS
    plt.figure(figsize=(10, 7))
    plt.plot(tt, innov_ok[:, 0], label="innov range (OK)")
    plt.plot(tt, np.rad2deg(innov_ok[:, 1]), label="innov bearing deg (OK)")
    plt.plot(tt, innov_bad[:, 0], label="innov range (Overconf)")
    plt.plot(tt, np.rad2deg(innov_bad[:, 1]), label="innov bearing deg (Overconf)")
    plt.plot(tt, nis_ok, label="NIS (OK)")
    plt.plot(tt, nis_bad, label="NIS (Overconf)")
    plt.grid(True, alpha=0.3)
    plt.xlabel("time [s]")
    plt.ylabel("innovation / NIS")
    plt.title(f"Day 30: Innovation + Consistency (NIS) ({name})")
    plt.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(f"results/day30_innovation_nis_{name}.png", dpi=200)
    plt.close()

    # Covariance trace (overconfidence collapse detector)
    plt.figure(figsize=(10, 6))
    plt.plot(tt, covtrace_ok, label="trace(P) OK")
    plt.plot(tt, covtrace_bad, label="trace(P) Overconfident")
    plt.grid(True, alpha=0.3)
    plt.xlabel("time [s]")
    plt.ylabel("trace(P)")
    plt.title(f"Day 30: Covariance Trace (Overconfidence) ({name})")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"results/day30_covtrace_{name}.png", dpi=200)
    plt.close()

    # quick numbers
    ok_rmse = float(np.sqrt(np.mean(err_ok[:, 0] ** 2 + err_ok[:, 1] ** 2)))
    bad_rmse = float(np.sqrt(np.mean(err_bad[:, 0] ** 2 + err_bad[:, 1] ** 2)))

    print(f"[{name}] RMSE position: OK={ok_rmse:.3f} m | Overconf={bad_rmse:.3f} m")
    print(f"[{name}] mean NIS:       OK={np.mean(nis_ok):.2f} | Overconf={np.mean(nis_bad):.2f}")
    print(f"[{name}] final trace(P): OK={covtrace_ok[-1]:.4f} | Overconf={covtrace_bad[-1]:.4f}\n")


def main():
    print("Running Day 30 EKF failure demo (corrected + hardened)...\n")

    run_scenario(name="mild", aggressive=False, seed=7)
    run_scenario(name="aggressive", aggressive=True, seed=7)

    print("Saved figures to results/")
    print("If overconf EKF doesn't diverge clearly on your run, make it more arrogant:")
    print("  - reduce Q_bad and/or R_bad further, OR")
    print("  - increase initial bias x0_est, OR")
    print("  - increase aggressive w_cmd amplitude.\n")


if __name__ == "__main__":
    main()