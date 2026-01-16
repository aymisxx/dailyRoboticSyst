import os
import numpy as np
import matplotlib.pyplot as plt

def simulate_pi_pd(
    a: float = 1.0,
    b: float = 1.0,
    Kp: float = 2.0,
    Ki: float = 1.0,
    Kd: float = 0.5,
    r: float = 1.0,
    dt: float = 0.01,
    T: float = 10.0,
    results_dir: str = "results",
):
    """
    Day 15: Demonstrate PI vs PD control on a first-order plant:
        x_dot = -a x + b u
    Tracks a step reference r.

    Notes:
    - Uses Euler integration.
    - Initializes prev_error for PD to avoid t=0 derivative kick.
    - Saves plots to results_dir and also shows them.
    """

    # Time vector
    time = np.arange(0.0, T, dt)

    # States
    x_pi = 0.0
    x_pd = 0.0

    # PI internal state
    integral_error = 0.0

    # PD internal state (avoid derivative kick on first step)
    prev_error_pd = r - x_pd

    # Logs
    x_pi_log = np.zeros_like(time)
    x_pd_log = np.zeros_like(time)
    u_pi_log = np.zeros_like(time)
    u_pd_log = np.zeros_like(time)

    # Sim loop
    for i, _t in enumerate(time):
        # PI control
        error_pi = r - x_pi
        integral_error += error_pi * dt
        u_pi = Kp * error_pi + Ki * integral_error

        # Plant update (PI)
        x_pi_dot = -a * x_pi + b * u_pi
        x_pi += x_pi_dot * dt

        # PD control
        error_pd = r - x_pd
        derivative_error = (error_pd - prev_error_pd) / dt
        u_pd = Kp * error_pd + Kd * derivative_error

        # Plant update (PD)
        x_pd_dot = -a * x_pd + b * u_pd
        x_pd += x_pd_dot * dt

        prev_error_pd = error_pd

        # Log
        x_pi_log[i] = x_pi
        x_pd_log[i] = x_pd
        u_pi_log[i] = u_pi
        u_pd_log[i] = u_pd

    # Make results dir
    os.makedirs(results_dir, exist_ok=True)

    # Plot: outputs
    plt.figure()
    plt.plot(time, x_pi_log, label="PI Output")
    plt.plot(time, x_pd_log, label="PD Output")
    plt.axhline(r, linestyle="--", label="Reference")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.title("Day 15: PI vs PD Output (First-Order Plant)")
    plt.legend()
    plt.grid(True)
    out_path = os.path.join(results_dir, "pi_vs_pd_output.png")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.show()

    # Plot: control effort
    plt.figure()
    plt.plot(time, u_pi_log, label="PI Control u(t)")
    plt.plot(time, u_pd_log, label="PD Control u(t)")
    plt.xlabel("Time (s)")
    plt.ylabel("Control effort")
    plt.title("Day 15: PI vs PD Control Effort")
    plt.legend()
    plt.grid(True)
    u_path = os.path.join(results_dir, "pi_vs_pd_control_effort.png")
    plt.savefig(u_path, dpi=200, bbox_inches="tight")
    plt.show()

    print(f"Saved plots to:\n- {out_path}\n- {u_path}")

if __name__ == "__main__":
    simulate_pi_pd()