# Day 04: Probability & Statistics for Systems
# Concepts:
# - Mean as central tendency
# - Variance / std as uncertainty
# - Covariance as correlation between signals
# - Noise as a modeled feature, not an error
#
# Run:
#   python day04_probability_statistics.py
#
# Outputs:
#   results/
#     01_noisy_signal_timeseries.png
#     02_histogram_with_gaussian.png
#     03_covariance_scatter.png

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt


# Helpers

def ensure_results_dir(path: str = "results") -> str:
    os.makedirs(path, exist_ok=True)
    return path


def gaussian_pdf(x: np.ndarray, mu: float, sigma: float) -> np.ndarray:
    return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(
        -0.5 * ((x - mu) / sigma) ** 2
    )



# Main

def main() -> None:
    results_dir = ensure_results_dir()

    np.random.seed(42)  # reproducibility

    # Time axis
    t = np.linspace(0, 10, 2000)

    # True underlying signal
    true_signal = np.sin(t)

    # Noise
    noise_std = 0.2
    noise = np.random.normal(0.0, noise_std, size=t.shape)

    # Measured signal
    measured_signal = true_signal + noise

    
    # Statistics
    
    mean_val = np.mean(measured_signal)
    var_val = np.var(measured_signal)
    std_val = np.std(measured_signal)

    # Create a second correlated signal
    noise_2 = 0.5 * noise + np.random.normal(0.0, 0.1, size=t.shape)
    signal_2 = np.cos(t) + noise_2

    covariance_matrix = np.cov(measured_signal, signal_2)

    print("Statistics Summary")
    print("------------------")
    print(f"Mean: {mean_val:.4f}")
    print(f"Variance: {var_val:.4f}")
    print(f"Std Dev: {std_val:.4f}")
    print("Covariance Matrix:")
    print(covariance_matrix)

    
    # Plot 1: Time series
    
    plt.figure()
    plt.plot(t, measured_signal, label="Measured (noisy)")
    plt.plot(t, true_signal, linestyle="--", label="True signal")
    plt.xlabel("Time")
    plt.ylabel("Signal")
    plt.title("Noisy Signal vs True Signal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "01_noisy_signal_timeseries.png"))
    plt.close()

    
    # Plot 2: Histogram + Gaussian
    
    plt.figure()
    count, bins, _ = plt.hist(
        measured_signal, bins=50, density=True, alpha=0.7, label="Histogram"
    )

    x_pdf = np.linspace(bins.min(), bins.max(), 400)
    pdf = gaussian_pdf(x_pdf, mean_val, std_val)
    plt.plot(x_pdf, pdf, label="Gaussian PDF")

    plt.xlabel("Signal value")
    plt.ylabel("Probability density")
    plt.title("Histogram with Gaussian Fit")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "02_histogram_with_gaussian.png"))
    plt.close()

    
    # Plot 3: Covariance scatter
    
    plt.figure()
    plt.scatter(measured_signal, signal_2, s=5, alpha=0.4)
    plt.xlabel("Signal 1")
    plt.ylabel("Signal 2")
    plt.title("Covariance Visualization")
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "03_covariance_scatter.png"))
    plt.close()


if __name__ == "__main__":
    main()