"""
Day 10: Stability (Continuous-Time LTI)
---------------------------------------
We analyze stability of x_dot = A x via eigenvalues of A, and simulate free response.

Rule of thumb (continuous-time):
- Re(eig) < 0  -> asymptotically stable
- Re(eig) > 0  -> unstable
- Re(eig) = 0  -> marginal (needs extra checks in general)

This script uses an MSD example:
x = [position, velocity]^T
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp



# Config

SAVE_DIR = "results"
os.makedirs(SAVE_DIR, exist_ok=True)

tol = 1e-9  # tolerance for eigenvalue real-part checks


# Define system (Mass-Spring-Damper)
# x_dot = A x

m = 1.0
c = 0.4
k = 4.0

A = np.array([
    [0.0, 1.0],
    [-k / m, -c / m]
], dtype=float)


# Eigenvalue analysis

eigvals = np.linalg.eigvals(A)

print("Day 10: Stability (CT LTI)")
print("--------------------------")
print(f"m={m}, c={c}, k={k}\n")
print("A =")
print(A)

print("\nEigenvalues of A:")
for lam in eigvals:
    print(f"  {lam.real:+.6f} {lam.imag:+.6f}j")


# Stability classification (practical)

real_parts = np.real(eigvals)

if np.all(real_parts < -tol):
    verdict = "Asymptotically Stable (all Re(λ) < 0)"
elif np.any(real_parts > tol):
    verdict = "Unstable (some Re(λ) > 0)"
else:
    verdict = "Marginal / Borderline (Re(λ) ≈ 0 for at least one eigenvalue)"

print(f"\nStability verdict: {verdict}")


# Simulate free response

def dynamics(t, x):
    return A @ x

t_span = (0.0, 10.0)
t_eval = np.linspace(t_span[0], t_span[1], 2000)
x0 = np.array([1.0, 0.0])  # initial displacement, zero velocity

sol = solve_ivp(
    fun=dynamics,
    t_span=t_span,
    y0=x0,
    t_eval=t_eval,
    rtol=1e-8,
    atol=1e-10
)

if not sol.success:
    raise RuntimeError(f"solve_ivp failed: {sol.message}")


# Plot + save

plt.figure(figsize=(8, 4))
plt.plot(sol.t, sol.y[0], label="Position x")
plt.plot(sol.t, sol.y[1], label="Velocity x_dot")
plt.xlabel("Time (s)")
plt.ylabel("State")
plt.title("Free Response: x_dot = A x")
plt.grid(True)
plt.legend()
plt.tight_layout()

out_path = os.path.join(SAVE_DIR, "free_response_states.png")
plt.savefig(out_path, dpi=200)
plt.show()

print(f"\nSaved plot: {out_path}")