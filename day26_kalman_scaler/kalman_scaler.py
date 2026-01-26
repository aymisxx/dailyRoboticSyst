"""
Day 26: Scalar Kalman Filter (1D)

Creates:
- results/kalman_scalar_plot.png
- results/kalman_scalar_data.csv
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def run_scalar_kalman(
    x_true: float = 50.0,
    Q: float = 1.0,
    R: float = 9.0,
    x0: float = 0.0,
    P0: float = 10.0,
    N: int = 30,
    seed: int = 26,
    results_dir: str = "results",
) -> dict:
    # sanity checks
    if Q < 0 or R <= 0:
        raise ValueError("Q must be >= 0 and R must be > 0.")
    if P0 < 0:
        raise ValueError("Initial covariance P0 must be >= 0.")
    if N <= 0:
        raise ValueError("N must be > 0.")

    rng = np.random.default_rng(seed)

    # results folder
    results_path = Path(results_dir)
    results_path.mkdir(parents=True, exist_ok=True)

    # init
    x_hat = float(x0)
    P = float(P0)

    estimates = np.zeros(N, dtype=float)
    measurements = np.zeros(N, dtype=float)
    K_list = np.zeros(N, dtype=float)
    P_list = np.zeros(N, dtype=float)
    P_pred_list = np.zeros(N, dtype=float)

    # loop
    for k in range(N):
        # measurement
        z = x_true + rng.normal(0.0, np.sqrt(R))

        # predict (random-walk model: x_k = x_{k-1} + w_k)
        x_pred = x_hat
        P_pred = P + Q

        # update
        K = P_pred / (P_pred + R)  # scalar Kalman gain
        x_hat = x_pred + K * (z - x_pred)
        P = (1.0 - K) * P_pred

        # store
        measurements[k] = z
        estimates[k] = x_hat
        K_list[k] = K
        P_list[k] = P
        P_pred_list[k] = P_pred

    # save CSV
    csv_path = results_path / "kalman_scalar_data.csv"
    header = "k,measurement_z,estimate_xhat,kalman_gain_K,P_pred,P"
    data = np.column_stack([np.arange(N), measurements, estimates, K_list, P_pred_list, P_list])
    np.savetxt(csv_path, data, delimiter=",", header=header, comments="", fmt="%.6f")

    # plot + save
    plot_path = results_path / "kalman_scalar_plot.png"
    plt.figure()
    plt.plot(estimates, label="Kalman estimate")
    plt.plot(measurements, ".", alpha=0.5, label="Measurements")
    plt.axhline(x_true, linestyle="--", label="True state")
    plt.title("Scalar Kalman Filter (1D)")
    plt.xlabel("Time step")
    plt.ylabel("State")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=200)
    plt.close()

    return {
        "x_true": x_true,
        "Q": Q,
        "R": R,
        "x0": x0,
        "P0": P0,
        "N": N,
        "seed": seed,
        "final_estimate": float(estimates[-1]),
        "final_covariance": float(P_list[-1]),
        "saved_plot": str(plot_path),
        "saved_csv": str(csv_path),
    }


if __name__ == "__main__":
    out = run_scalar_kalman()

    print("Day 26 complete.")
    print(f"- True state (hidden): x = {out['x_true']:.2f}")
    print(f"- Q (process var):     {out['Q']:.2f}")
    print(f"- R (meas var):        {out['R']:.2f}")
    print(f"- Final estimate:      x̂ = {out['final_estimate']:.3f}")
    print(f"- Final covariance:    P  = {out['final_covariance']:.3f}")
    print(f"- Saved plot:          {out['saved_plot']}")
    print(f"- Saved data:          {out['saved_csv']}")