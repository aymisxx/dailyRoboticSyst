"""
Day 02: Calculus for Dynamics (Numerical)
-----------------------------------------
Goal:
- See derivatives as "rate of change"
- See integrals as "accumulation / memory"
- Implement both numerically in simple, readable Python

Run:
  python numerical_calculus.py

Outputs:
  results/
    01_signal_and_derivative.png
    02_velocity_and_position_integral.png
"""

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt



# Helpers

def ensure_results_dir(path: str = "results") -> str:
    """Create results/ folder if it doesn't exist, return its path."""
    os.makedirs(path, exist_ok=True)
    return path


def numerical_derivative(x: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Compute dx/dt numerically.
    Uses numpy.gradient which is stable + returns same length as x.
    """
    return np.gradient(x, t)


def euler_integral(dx: np.ndarray, t: np.ndarray, x0: float = 0.0) -> np.ndarray:
    """
    Compute x(t) from dx/dt via Euler integration:
      x[k+1] = x[k] + dx[k] * dt

    Returns x with the same length as dx and t.
    """
    x = np.zeros_like(dx, dtype=float)
    x[0] = x0

    for k in range(len(t) - 1):
        dt = t[k + 1] - t[k]
        x[k + 1] = x[k] + dx[k] * dt

    return x



# Main demo

def main() -> None:
    results_dir = ensure_results_dir("results")

    # Time axis
    t_end = 10.0
    n = 2000
    t = np.linspace(0.0, t_end, n)

    # Example signal: x(t) = sin(2π f t) + small trend
    
    f = 0.7  # Hz
    x = np.sin(2.0 * np.pi * f * t) + 0.15 * t

    # 1) Derivative: dx/dt
    dx_dt = numerical_derivative(x, t)

    # Plot: signal + its derivative
    plt.figure()
    plt.plot(t, x, label="x(t) (signal)")
    plt.plot(t, dx_dt, label="dx/dt (numerical derivative)")
    plt.xlabel("time (s)")
    plt.ylabel("value")
    plt.title("Derivative = Rate of Change")
    plt.legend()
    plt.grid(True)
    out1 = os.path.join(results_dir, "01_signal_and_derivative.png")
    plt.savefig(out1, dpi=200, bbox_inches="tight")
    plt.close()

    # 2) Building position from velocity (classic physics)
    # Let velocity v(t) be a smooth signal:
    v = 1.5 * np.cos(2.0 * np.pi * 0.4 * t) + 0.3  # includes bias (0.3)
    # Integrate velocity -> position:
    p = euler_integral(v, t, x0=0.0)

    # Plot: velocity + integrated position
    plt.figure()
    plt.plot(t, v, label="v(t) (velocity)")
    plt.plot(t, p, label="p(t) = ∫ v(t) dt (Euler integral)")
    plt.xlabel("time (s)")
    plt.ylabel("value")
    plt.title("Integral = Accumulation / Memory")
    plt.legend()
    plt.grid(True)
    out2 = os.path.join(results_dir, "02_velocity_and_position_integral.png")
    plt.savefig(out2, dpi=200, bbox_inches="tight")
    plt.close()

    # Console summary
    print("Saved:")
    print(" -", out1)
    print(" -", out2)
    print("\nNotes:")
    print(" - Derivative curve reacts instantly to changes in x(t).")
    print(" - Integral curve keeps accumulating, so any bias in v(t) causes drift.")


if __name__ == "__main__":
    main()