"""
Day 14: Proportional Control (P)

Plant: Mass-Spring-Damper (2nd order)
    m x_ddot + c x_dot + k x = u

State-space:
    x1 = x (position)
    x2 = x_dot (velocity)
    x1_dot = x2
    x2_dot = (1/m) * (u - c*x2 - k*x1)

Controller (P):
    u = Kp * (r - y), where y = x

We simulate a step reference r(t) and compare responses for multiple Kp values.
Outputs:
    results/step_response_all_kp.png
    results/error_all_kp.png
    results/control_all_kp.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt


def rk4_step(f, t, x, dt, u):
    """One RK4 integration step for x_dot = f(t, x, u)."""
    k1 = f(t, x, u)
    k2 = f(t + 0.5 * dt, x + 0.5 * dt * k1, u)
    k3 = f(t + 0.5 * dt, x + 0.5 * dt * k2, u)
    k4 = f(t + dt, x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def make_results_dir():
    outdir = "results"
    os.makedirs(outdir, exist_ok=True)
    return outdir


def simulate_p_control(
    m=1.0, c=1.0, k=4.0,
    Kp=5.0,
    r_step=1.0,
    t_final=8.0,
    dt=0.001,
    u_limit=None
):
    """
    Simulate closed-loop with P controller on MSD plant.

    u_limit: None or tuple (umin, umax) for saturation
    """
    n = int(t_final / dt) + 1
    t = np.linspace(0.0, t_final, n)

    # State: [x, x_dot]
    x = np.zeros(2, dtype=float)

    # Logs
    y_log = np.zeros(n)
    e_log = np.zeros(n)
    u_log = np.zeros(n)

    def dynamics(_t, state, u):
        x1, x2 = state
        x1_dot = x2
        x2_dot = (u - c * x2 - k * x1) / m
        return np.array([x1_dot, x2_dot], dtype=float)

    for i in range(n):
        y = x[0]              # output is position
        r = r_step            # step reference
        e = r - y
        u = Kp * e

        # Optional saturation (helps visualize "too much Kp" realism)
        if u_limit is not None:
            u = np.clip(u, u_limit[0], u_limit[1])

        y_log[i] = y
        e_log[i] = e
        u_log[i] = u

        if i < n - 1:
            x = rk4_step(dynamics, t[i], x, dt, u)

    return t, y_log, e_log, u_log


def main():
    outdir = make_results_dir()

    # Plant parameters (tweak if you want different vibe)
    m = 1.0
    c = 1.0
    k = 4.0

    # P gains to compare
    kp_list = [2.0, 8.0, 25.0]

    # Simulation settings
    t_final = 8.0
    dt = 0.001
    r_step = 1.0

    # Optional actuator saturation
    u_limit = (-50.0, 50.0)

    sims = []
    for Kp in kp_list:
        t, y, e, u = simulate_p_control(
            m=m, c=c, k=k,
            Kp=Kp,
            r_step=r_step,
            t_final=t_final,
            dt=dt,
            u_limit=u_limit
        )
        sims.append((Kp, t, y, e, u))

    # Plot 1: Output vs Reference
    plt.figure()
    for Kp, t, y, _, _ in sims:
        plt.plot(t, y, label=f"Kp={Kp:g}")
    plt.plot(t, r_step * np.ones_like(t), linestyle="--", label="reference (step)")
    plt.xlabel("Time [s]")
    plt.ylabel("Position y(t)")
    plt.title("Closed-loop Step Response (P Control)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "step_response_all_kp.png"), dpi=200)

    # Plot 2: Error
    plt.figure()
    for Kp, t, _, e, _ in sims:
        plt.plot(t, e, label=f"Kp={Kp:g}")
    plt.xlabel("Time [s]")
    plt.ylabel("Error e(t) = r - y")
    plt.title("Tracking Error (P Control)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "error_all_kp.png"), dpi=200)

    # Plot 3: Control Effort
    plt.figure()
    for Kp, t, _, _, u in sims:
        plt.plot(t, u, label=f"Kp={Kp:g}")
    if u_limit is not None:
        plt.axhline(u_limit[1], linestyle="--")
        plt.axhline(u_limit[0], linestyle="--")
    plt.xlabel("Time [s]")
    plt.ylabel("Control input u(t)")
    plt.title("Control Effort (P Control)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "control_all_kp.png"), dpi=200)

    print("Day 14 done. Saved plots to:", outdir)
    print("   - step_response_all_kp.png")
    print("   - error_all_kp.png")
    print("   - control_all_kp.png")
    if u_limit is not None:
        print(f"   (Note: actuator saturation enabled at u in [{u_limit[0]}, {u_limit[1]}])")


if __name__ == "__main__":
    main()