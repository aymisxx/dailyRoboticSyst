"""
Day 23: Sensors & Noise (Systems-first)

Minimal simulation showing how a "true" 1D motion gets corrupted by:
- white noise
- bias
- drift (random walk)
- quantization
- latency
- dropouts
- outliers

Outputs:
- results/sensor_signals.png
- results/error_histograms.png
- results/telemetry.csv

Run:
    python day23_sensors_and_noise.py
"""

from __future__ import annotations
import os
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt



# Config

@dataclass
class SimConfig:
    dt: float = 0.01
    T: float = 10.0
    seed: int = 23

    # "Sensor" knobs
    white_sigma: float = 0.20              # std dev of white noise
    bias: float = 0.60                     # constant offset
    drift_sigma_per_sqrt_s: float = 0.06   # random walk intensity (per sqrt(second))
    quant_step: float = 0.10               # quantization step
    latency_s: float = 0.20                # latency in seconds

    dropout_prob: float = 0.03             # probability sample is missing
    outlier_prob: float = 0.01             # probability sample is a spike
    outlier_sigma: float = 2.5             # spike magnitude scale

    results_dir: str = "results"



# Utilities

def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def quantize(x: np.ndarray, step: float) -> np.ndarray:
    """Quantize with step size; preserves NaNs."""
    if step <= 0:
        return x.copy()
    return step * np.round(x / step)


def apply_latency(x: np.ndarray, lag_steps: int) -> np.ndarray:
    """
    Pure delay. First lag_steps values are held at the first finite sample (or 0 if none).
    This avoids NaN-propagation when the first sample is missing.
    """
    if lag_steps <= 0:
        return x.copy()

    y = np.empty_like(x)

    finite_idx = np.flatnonzero(np.isfinite(x))
    hold = x[finite_idx[0]] if finite_idx.size else 0.0

    y[:lag_steps] = hold
    y[lag_steps:] = x[:-lag_steps]
    return y


def add_dropouts(x: np.ndarray, p: float, rng: np.random.Generator) -> np.ndarray:
    """Replace some samples with NaN."""
    y = x.copy()
    if p <= 0:
        return y
    mask = rng.random(len(x)) < p
    y[mask] = np.nan
    return y


def add_outliers(x: np.ndarray, p: float, sigma: float, rng: np.random.Generator) -> np.ndarray:
    """Add occasional spikes."""
    y = x.copy()
    if p <= 0:
        return y
    mask = rng.random(len(x)) < p
    spikes = rng.normal(0.0, sigma, size=len(x))
    y[mask] = y[mask] + spikes[mask]
    return y



# Main simulation

def main(cfg: SimConfig) -> None:
    rng = np.random.default_rng(cfg.seed)
    ensure_dir(cfg.results_dir)

    # Time axis
    N = int(np.round(cfg.T / cfg.dt)) + 1
    t = np.linspace(0.0, cfg.T, N)

    # True motion: 1D position signal with mixed frequencies
    x_true = 2.0 * np.sin(2 * np.pi * 0.35 * t) + 0.6 * np.sin(2 * np.pi * 1.6 * t)

    # Sensor models

    # 1) White noise sensor
    y_white = x_true + rng.normal(0.0, cfg.white_sigma, size=N)

    # 2) Bias + white noise
    y_bias = x_true + cfg.bias + rng.normal(0.0, cfg.white_sigma, size=N)

    # 3) Drift (random walk) + white noise
    drift_incr_sigma = cfg.drift_sigma_per_sqrt_s * np.sqrt(cfg.dt)
    drift = np.cumsum(rng.normal(0.0, drift_incr_sigma, size=N))
    y_drift = x_true + drift + rng.normal(0.0, cfg.white_sigma * 0.5, size=N)

    # 4) Quantized sensor (ADC resolution effect)
    y_quant = quantize(
        x_true + rng.normal(0.0, cfg.white_sigma * 0.35, size=N),
        cfg.quant_step
    )

    # 5) Latency sensor (delayed measurement) + noise
    lag_steps = int(np.round(cfg.latency_s / cfg.dt))
    y_late = apply_latency(x_true, lag_steps) + rng.normal(0.0, cfg.white_sigma * 0.25, size=N)

    # 6) Dropout sensor (missing samples) + noise
    y_drop = x_true + rng.normal(0.0, cfg.white_sigma * 0.25, size=N)
    y_drop = add_dropouts(y_drop, cfg.dropout_prob, rng)

    # 7) Outlier sensor (spiky) + noise
    y_out = x_true + rng.normal(0.0, cfg.white_sigma * 0.25, size=N)
    y_out = add_outliers(y_out, cfg.outlier_prob, cfg.outlier_sigma, rng)

    # 8) Combo sensor (more "realistic-ish" pipeline)
    # Order chosen for intuition: latency -> quantization -> outliers -> dropouts (last)
    y_combo = x_true.copy()
    y_combo = y_combo + 0.25 * cfg.bias
    y_combo = y_combo + 0.6 * drift
    y_combo = y_combo + rng.normal(0.0, cfg.white_sigma * 0.35, size=N)
    y_combo = apply_latency(y_combo, lag_steps)
    y_combo = quantize(y_combo, cfg.quant_step)
    y_combo = add_outliers(y_combo, cfg.outlier_prob, 0.9 * cfg.outlier_sigma, rng)
    y_combo = add_dropouts(y_combo, cfg.dropout_prob, rng)

    # Save telemetry CSV (NaNs will be written as "nan")
    csv_path = os.path.join(cfg.results_dir, "telemetry.csv")
    header = "t,x_true,y_white,y_bias,y_drift,y_quant,y_late,y_drop,y_out,y_combo"
    data = np.column_stack([t, x_true, y_white, y_bias, y_drift, y_quant, y_late, y_drop, y_out, y_combo])
    np.savetxt(csv_path, data, delimiter=",", header=header, comments="")

    # Plot signals
    fig = plt.figure(figsize=(12, 8))

    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(t, x_true, linewidth=2, label="true")
    ax1.plot(t, y_white, alpha=0.8, label="white")
    ax1.plot(t, y_bias, alpha=0.8, label="bias+white")
    ax1.plot(t, y_drift, alpha=0.8, label="drift+white")
    ax1.set_title("Signals (true vs sensors)")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("position")
    ax1.grid(True, alpha=0.25)
    ax1.legend(ncols=2)

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(t, x_true, linewidth=2, label="true")
    ax2.plot(t, y_quant, alpha=0.8, label="quantized")
    ax2.plot(t, y_late, alpha=0.8, label=f"latency ({cfg.latency_s:.2f}s)")
    ax2.plot(t, y_drop, alpha=0.8, label="dropouts (NaNs)")
    ax2.plot(t, y_out, alpha=0.8, label="outliers")
    ax2.plot(t, y_combo, alpha=0.9, label="combo sensor")
    ax2.set_title("More sensor failure modes")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("position")
    ax2.grid(True, alpha=0.25)
    ax2.legend(ncols=2)

    fig.tight_layout()
    out1 = os.path.join(cfg.results_dir, "sensor_signals.png")
    fig.savefig(out1, dpi=200)
    plt.close(fig)

    # Plot error histograms (ignore NaNs)
    sensors = {
        "white": y_white,
        "bias": y_bias,
        "drift": y_drift,
        "quant": y_quant,
        "late": y_late,
        "drop": y_drop,
        "out": y_out,
        "combo": y_combo,
    }

    all_err = []
    for y in sensors.values():
        e = y - x_true
        all_err.append(e[np.isfinite(e)])
    all_err = np.concatenate(all_err) if all_err else np.array([0.0])

    lo, hi = np.percentile(all_err, [0.5, 99.5])
    bins = np.linspace(lo, hi, 60)

    fig2 = plt.figure(figsize=(12, 8))
    ax = fig2.add_subplot(1, 1, 1)
    for k, y in sensors.items():
        e = (y - x_true)
        e = e[np.isfinite(e)]
        ax.hist(e, bins=bins, alpha=0.35, density=True, label=k)

    ax.set_title("Error distributions (sensor - true)")
    ax.set_xlabel("error")
    ax.set_ylabel("density")
    ax.grid(True, alpha=0.25)
    ax.legend(ncols=4)
    fig2.tight_layout()
    out2 = os.path.join(cfg.results_dir, "error_histograms.png")
    fig2.savefig(out2, dpi=200)
    plt.close(fig2)

    # Quick numeric summary
    print("Saved:")
    print(" -", out1)
    print(" -", out2)
    print(" -", csv_path)
    print("\nQuick stats (RMSE, ignoring NaNs):")
    for k, y in sensors.items():
        e = (y - x_true)
        e = e[np.isfinite(e)]
        rmse = float(np.sqrt(np.mean(e**2))) if e.size else float("nan")
        print(f"  {k:>6s}: RMSE = {rmse:.4f}")


if __name__ == "__main__":
    cfg = SimConfig()
    main(cfg)