import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from pathlib import Path

# Robust paths: based on this file's location, not where you run from
THIS_DIR = Path(__file__).resolve().parent
ROOT_DIR = THIS_DIR.parent
RESULTS_DIR = ROOT_DIR / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

from mass_spring_damper import MassSpringDamper  # noqa: E402

# System parameters
m = 1.0
c = 0.5
k = 4.0
system = MassSpringDamper(m, c, k)

# Time span
t_span = (0.0, 10.0)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Initial state: [position, velocity]
x0 = [1.0, 0.0]

def simulate(u_const: float):
    sol = solve_ivp(
        fun=lambda t, y: system.dynamics(t, y, u=u_const),
        t_span=t_span,
        y0=x0,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-8,
        atol=1e-10,
    )
    if not sol.success:
        raise RuntimeError(f"ODE solver failed: {sol.message}")
    return sol

# Free response (u = 0)
sol_free = simulate(u_const=0.0)

# Forced response (constant force u = 1)
sol_forced = simulate(u_const=1.0)

# Plots
def save_plot(path: Path):
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()

# Free response
plt.figure()
plt.plot(sol_free.t, sol_free.y[0])
plt.xlabel("Time [s]")
plt.ylabel("Position x [m]")
plt.title("Free Response (u = 0)")
save_plot(RESULTS_DIR / "free_response.png")

# Forced response
plt.figure()
plt.plot(sol_forced.t, sol_forced.y[0])
plt.xlabel("Time [s]")
plt.ylabel("Position x [m]")
plt.title("Forced Response (u = 1)")
save_plot(RESULTS_DIR / "forced_response.png")

# Phase plot (free response)
plt.figure()
plt.plot(sol_free.y[0], sol_free.y[1])
plt.xlabel("Position x [m]")
plt.ylabel("Velocity x_dot [m/s]")
plt.title("Phase Plot (Free Response)")
save_plot(RESULTS_DIR / "phase_plot.png")

print(f"Saved results to: {RESULTS_DIR}")