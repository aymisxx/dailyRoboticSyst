import numpy as np
import matplotlib
matplotlib.use("Agg")  # safe for WSL/headless
import matplotlib.pyplot as plt
from pathlib import Path

def rot2(theta: float) -> np.ndarray:
    """2D rotation matrix."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])

def simulate_case(
    *,
    seed: int,
    dt: float,
    T: float,
    g: float,
    gyro_bias_deg_s: float,
    gyro_noise_deg_s: float,
    accel_bias: np.ndarray,
    accel_noise_std: float
):
    """
    Stationary robot IMU demo in 2D (yaw only).
    Shows how gyro bias -> yaw drift -> wrong gravity removal -> fake horizontal accel -> position drift.
    """
    rng = np.random.default_rng(seed)
    N = int(T / dt)
    t = np.arange(N) * dt

    # Truth: robot is perfectly stationary (no rotation, no translation)
    omega_true = np.zeros(N)  # rad/s (yaw rate true)

    # Gyro: measured omega = true + bias + noise
    gyro_bias = np.deg2rad(gyro_bias_deg_s)      # rad/s
    gyro_noise_std = np.deg2rad(gyro_noise_deg_s)  # rad/s
    omega_meas = omega_true + gyro_bias + rng.normal(0.0, gyro_noise_std, size=N)

    # Integrate gyro -> yaw estimate
    theta_est = np.cumsum(omega_meas) * dt  # rad

    # Accelerometer (specific force) in BODY frame
    # For this demo, assume gravity shows up as +g along +y_body at rest.
    # (Sign convention doesn't matter as long as we're consistent.)
    a_meas_body = np.zeros((N, 2))
    for k in range(N):
        noise = rng.normal(0.0, accel_noise_std, size=2)
        a_meas_body[k] = np.array([0.0, g]) + accel_bias + noise

    # Rotate accel into WORLD frame using *estimated* yaw
    a_world_est = np.zeros((N, 2))
    for k in range(N):
        R = rot2(theta_est[k])
        a_world_est[k] = R @ a_meas_body[k]

    # Naive gravity removal in WORLD frame
    # If yaw estimate is wrong, this subtraction is wrong -> fake linear accel appears.
    a_lin_world = a_world_est - np.array([0.0, g])

    # Integrate to velocity and position (naive dead reckoning)
    v = np.cumsum(a_lin_world, axis=0) * dt
    p = np.cumsum(v, axis=0) * dt

    return t, theta_est, a_lin_world, p

def save_plots(prefix: str, out_dir: Path, t, theta_est, a_lin_world, p):
    # 1) yaw drift
    plt.figure()
    plt.plot(t, np.rad2deg(theta_est))
    plt.xlabel("time (s)")
    plt.ylabel("yaw (deg)")
    plt.title(f"{prefix}: Gyro integration -> yaw drift (stationary robot)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / f"{prefix}_yaw_drift.png", dpi=200)
    plt.close()

    # 2) fake horizontal accel
    plt.figure()
    plt.plot(t, a_lin_world[:, 0])
    plt.xlabel("time (s)")
    plt.ylabel("m/s^2")
    plt.title(f"{prefix}: Wrong gravity removal -> fake horizontal acceleration (x)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / f"{prefix}_accel_world_x.png", dpi=200)
    plt.close()

    # 3) position drift
    plt.figure()
    plt.plot(p[:, 0], p[:, 1])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(f"{prefix}: Double integration drift (stationary robot!)")
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / f"{prefix}_position_drift.png", dpi=200)
    plt.close()

def main():
    out_dir = Path("results")
    out_dir.mkdir(exist_ok=True)

    # Shared sim settings
    dt = 0.01
    T = 30.0
    g = 9.81

    # Noise/bias settings (small but realistic-ish)
    gyro_noise_deg_s = 0.02
    accel_noise_std = 0.03
    accel_bias = np.array([0.02, -0.01])  # m/s^2

    # Case A: Bias ON (the villain)
    tA, thetaA, aA, pA = simulate_case(
        seed=7,
        dt=dt, T=T, g=g,
        gyro_bias_deg_s=0.5,          # <-- key bias
        gyro_noise_deg_s=gyro_noise_deg_s,
        accel_bias=accel_bias,
        accel_noise_std=accel_noise_std
    )
    save_plots("bias_on", out_dir, tA, thetaA, aA, pA)

    # Case B: Bias OFF (control experiment)
    tB, thetaB, aB, pB = simulate_case(
        seed=7,  # same seed to isolate effect of bias
        dt=dt, T=T, g=g,
        gyro_bias_deg_s=0.0,          # <-- remove bias
        gyro_noise_deg_s=gyro_noise_deg_s,
        accel_bias=accel_bias,
        accel_noise_std=accel_noise_std
    )
    save_plots("bias_off", out_dir, tB, thetaB, aB, pB)

    # Sanity prints
    print("Sanity Check")
    print(f"Bias ON final yaw:  {np.rad2deg(thetaA[-1]):.2f} deg (expected ~15 deg over 30s for 0.5 deg/s)")
    print(f"Bias OFF final yaw: {np.rad2deg(thetaB[-1]):.2f} deg (should be much smaller; noise-only random walk)")

    print(f"Bias ON final pos:  x={pA[-1,0]:.2f} m, y={pA[-1,1]:.2f} m")
    print(f"Bias OFF final pos: x={pB[-1,0]:.2f} m, y={pB[-1,1]:.2f} m")
    print("Saved 6 plots in ./results/")

if __name__ == "__main__":
    main()