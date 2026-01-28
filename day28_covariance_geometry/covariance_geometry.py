import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_covariance_ellipse(mean, cov, ax, n_std=2.0, **kwargs):
    """
    Plot an n-std covariance ellipse for a 2D Gaussian.
    """
    eigvals, eigvecs = np.linalg.eigh(cov)

    order = eigvals.argsort()[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    width, height = 2 * n_std * np.sqrt(eigvals)

    ellipse = Ellipse(
        xy=mean,
        width=width,
        height=height,
        angle=angle,
        fill=False,
        **kwargs
    )
    ax.add_patch(ellipse)


# State mean and covariance

mean = np.array([2.0, 1.0])

cov = np.array([
    [0.8, 0.6],
    [0.6, 1.2]
])


# Plot

fig, ax = plt.subplots(figsize=(6, 6))

ax.scatter(mean[0], mean[1], c='red', label='Mean')
plot_covariance_ellipse(mean, cov, ax, n_std=1.0, edgecolor='blue', label='1σ')
plot_covariance_ellipse(mean, cov, ax, n_std=2.0, edgecolor='green', linestyle='--', label='2σ')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Day 28: Covariance as Geometry')
ax.axis('equal')
ax.grid(True)
ax.legend()


# Save figure

os.makedirs("results", exist_ok=True)
plt.tight_layout()
plt.savefig("results/day28_covariance_ellipse.png", dpi=300)
plt.close()