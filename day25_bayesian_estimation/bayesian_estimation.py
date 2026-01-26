"""
Day 25: Bayesian Estimation Intuition (1D Grid Belief Update)

Concept:
Posterior p(x|z) ∝ Likelihood p(z|x) * Prior p(x)

We model a 1D discrete world: x ∈ {0,1,...,N-1}
Measurement model: z = x + noise, noise ~ N(0, sigma^2)
So: p(z|x) = Normal(z; mean=x, std=sigma)

Outputs:
  ./results/bayes_update_plot.png
  ./results/bayes_update_data.csv

Run:
  python bayesian_estimation.py
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt


def normalize(p: np.ndarray, eps: float = 1e-15) -> np.ndarray:
    """Normalize a discrete distribution safely."""
    s = float(np.sum(p))
    if s < eps:
        return np.ones_like(p, dtype=float) / len(p)
    return p / s


def gaussian_pdf(x: np.ndarray, mu: float, sigma: float) -> np.ndarray:
    """Proper Gaussian PDF evaluated at x."""
    sigma = max(float(sigma), 1e-9)
    coef = 1.0 / (np.sqrt(2.0 * np.pi) * sigma)
    return coef * np.exp(-0.5 * ((x - mu) / sigma) ** 2)


def make_prior(xs: np.ndarray, kind: str = "gaussian", mu: float = 20.0, sigma: float = 6.0) -> np.ndarray:
    """Create a prior p(x)."""
    if kind == "uniform":
        return np.ones_like(xs, dtype=float) / len(xs)
    if kind == "gaussian":
        return normalize(gaussian_pdf(xs, mu=mu, sigma=sigma))
    raise ValueError("prior kind must be 'uniform' or 'gaussian'")


def main():
    # Config
    N = 60                      # number of discrete positions
    true_x = 35                 # hidden true state (unknown to estimator)
    measurement_sigma = 4.0     # sensor noise std-dev
    prior_kind = "gaussian"     # "uniform" or "gaussian"
    prior_mu = 18.0             # prior mean (initial guess)
    prior_sigma = 8.0           # prior std-dev (initial uncertainty)
    seed = 7                    # RNG seed for reproducibility

    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)

    rng = np.random.default_rng(seed)
    xs = np.arange(N, dtype=float)

    # Simulate measurement
    # z = true_x + noise
    z = float(true_x + rng.normal(0.0, measurement_sigma))

    # Build prior p(x)
    prior = make_prior(xs, kind=prior_kind, mu=prior_mu, sigma=prior_sigma)

    # Build likelihood p(z|x)
    # p(z|x) = Normal(z; mean=x, std=measurement_sigma)
    # Evaluate likelihood for each discrete state x by plugging z into a Gaussian centered at x.
    likelihood = gaussian_pdf(z, mu=xs, sigma=measurement_sigma)  # vectorized over xs

    # Bayes update
    posterior = normalize(prior * likelihood)

    # MAP estimate (most probable discrete state)
    map_x = int(np.argmax(posterior))

    # Save CSV
    csv_path = os.path.join(results_dir, "bayes_update_data.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "prior", "likelihood", "posterior"])
        for i in range(N):
            w.writerow([i, float(prior[i]), float(likelihood[i]), float(posterior[i])])

    # Plot
    plt.figure()
    plt.plot(xs, prior, label="Prior p(x)")
    plt.plot(xs, normalize(likelihood), label="Likelihood p(z|x) (scaled)")  # scaled for visual comparability
    plt.plot(xs, posterior, label="Posterior p(x|z)")

    plt.axvline(true_x, linestyle="--", linewidth=1, label=f"True x = {true_x}")
    plt.axvline(z, linestyle=":", linewidth=1, label=f"Measurement z = {z:.2f}")
    plt.axvline(map_x, linestyle="-.", linewidth=1, label=f"MAP estimate = {map_x}")

    plt.title("Bayesian Estimation: Prior × Likelihood → Posterior")
    plt.xlabel("Position x")
    plt.ylabel("Probability (discrete)")
    plt.legend()
    plt.tight_layout()

    fig_path = os.path.join(results_dir, "bayes_update_plot.png")
    plt.savefig(fig_path, dpi=200)
    plt.close()

    # Summary
    print("Day 25 complete.")
    print(f"- True state (hidden): x = {true_x}")
    print(f"- Measurement observed: z = {z:.2f} (sigma={measurement_sigma})")
    print(f"- MAP estimate:        x̂ = {map_x}")
    print(f"- Saved plot:          {fig_path}")
    print(f"- Saved data:          {csv_path}")


if __name__ == "__main__":
    main()