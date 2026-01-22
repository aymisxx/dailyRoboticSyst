"""
Day 22: Control Failure Modes

Demo: Integrator Windup under Actuator Saturation + Anti-Windup fix.

Plant: 1D mass (double integrator)
    x = [position, velocity]
    u = force / acceleration command (bounded)

Controller: PID on position error
    u = Kp*e + Ki*∫e dt + Kd*de/dt
Actuator saturation:
    u_sat = clip(u, u_min, u_max)

Failure mode:
    - When saturated, the integrator keeps accumulating error -> windup
    - When saturation releases, controller overreacts -> overshoot, long settling

Fix:
    - Anti-windup via back-calculation:
        I_dot = e + (u_sat - u_unsat)/Kaw

Outputs (saved to ./results):
    day22_position.png
    day22_velocity.png
    day22_control.png
    day22_integrator.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt



# Utilities

def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def clip(u: float, u_min: float, u_max: float) -> float:
    return float(np.clip(u, u_min, u_max))



# Simulation

def simulate_pid_windup_vs_antiwindup(
    dt: float = 0.01,
    T: float = 10.0,
    x0: np.ndarray | None = None,
    x_ref: np.ndarray | None = None,
    # PID gains (tuned to show windup clearly under saturation)
    Kp: float = 18.0,
    Ki: float = 10.0,
    Kd: float = 6.0,
    # Actuator limits
    u_min: float = -1.0,
    u_max: float = 1.0,
    # Anti-windup back-calculation gain (bigger = faster unwinding)
    Kaw: float = 5.0,
):
    """
    Runs two simulations:
      (1) PID + saturation (windup)
      (2) PID + saturation + anti-windup back-calculation

    Returns dict of time-series.
    """
    if x0 is None:
        x0 = np.array([8.0, 0.0], dtype=float)  # start far away
    if x_ref is None:
        x_ref = np.array([0.0, 0.0], dtype=float)

    steps = int(T / dt)
    t = np.arange(steps + 1) * dt

    # State storages: [pos, vel]
    x_w = np.zeros((steps + 1, 2), dtype=float)  # windup case
    x_aw = np.zeros((steps + 1, 2), dtype=float)  # anti-windup case

    # Control storages
    u_unsat_w = np.zeros(steps, dtype=float)
    u_sat_w = np.zeros(steps, dtype=float)

    u_unsat_aw = np.zeros(steps, dtype=float)
    u_sat_aw = np.zeros(steps, dtype=float)

    # Integrator storages (integral state)
    I_w = np.zeros(steps + 1, dtype=float)
    I_aw = np.zeros(steps + 1, dtype=float)

    # Derivative term uses measured error difference
    e_prev_w = float(x_ref[0] - x0[0])
    e_prev_aw = float(x_ref[0] - x0[0])

    # Init
    x_w[0] = x0.copy()
    x_aw[0] = x0.copy()

    # Dynamics: double integrator
    # pos_dot = vel
    # vel_dot = u
    for k in range(steps):

        # WINDUP CASE (no anti-windup)

        pos_w, vel_w = x_w[k]
        e_w = float(x_ref[0] - pos_w)
        de_w = (e_w - e_prev_w) / dt

        # Integrator accumulates regardless of saturation (this is the bug)
        I_w[k + 1] = I_w[k] + e_w * dt

        u_w = Kp * e_w + Ki * I_w[k + 1] + Kd * de_w
        u_w_sat = clip(u_w, u_min, u_max)

        u_unsat_w[k] = u_w
        u_sat_w[k] = u_w_sat

        # Plant integration (Euler)
        pos_w_next = pos_w + vel_w * dt
        vel_w_next = vel_w + u_w_sat * dt
        x_w[k + 1] = [pos_w_next, vel_w_next]

        e_prev_w = e_w


        # ANTI-WINDUP CASE (back-calculation)

        pos_aw, vel_aw = x_aw[k]
        e_aw = float(x_ref[0] - pos_aw)
        de_aw = (e_aw - e_prev_aw) / dt

        # First compute unsaturated control with current integrator
        u_aw_unsat = Kp * e_aw + Ki * I_aw[k] + Kd * de_aw
        u_aw_sat = clip(u_aw_unsat, u_min, u_max)

        # Back-calculation: drives integrator to be consistent with saturation
        # I_dot = e + (u_sat - u_unsat)/Kaw
        I_dot = e_aw + (u_aw_sat - u_aw_unsat) / max(Kaw, 1e-9)
        I_aw[k + 1] = I_aw[k] + I_dot * dt

        # Recompute control using updated integrator (optional but stable)
        u_aw = Kp * e_aw + Ki * I_aw[k + 1] + Kd * de_aw
        u_aw_sat2 = clip(u_aw, u_min, u_max)

        u_unsat_aw[k] = u_aw
        u_sat_aw[k] = u_aw_sat2

        # Plant
        pos_aw_next = pos_aw + vel_aw * dt
        vel_aw_next = vel_aw + u_aw_sat2 * dt
        x_aw[k + 1] = [pos_aw_next, vel_aw_next]

        e_prev_aw = e_aw

    return {
        "t": t,
        "x_w": x_w,
        "x_aw": x_aw,
        "u_unsat_w": u_unsat_w,
        "u_sat_w": u_sat_w,
        "u_unsat_aw": u_unsat_aw,
        "u_sat_aw": u_sat_aw,
        "I_w": I_w,
        "I_aw": I_aw,
        "params": {
            "dt": dt,
            "T": T,
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "u_min": u_min,
            "u_max": u_max,
            "Kaw": Kaw,
            "x0": x0,
            "x_ref": x_ref,
        },
    }



# Plotting

def make_plots(data: dict, results_dir: str) -> None:
    ensure_dir(results_dir)

    t = data["t"]
    x_w = data["x_w"]
    x_aw = data["x_aw"]

    # time for control is one shorter
    t_u = t[:-1]

    # Position
    plt.figure()
    plt.plot(t, x_w[:, 0], label="PID + saturation (windup)")
    plt.plot(t, x_aw[:, 0], label="PID + saturation + anti-windup")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.title("Day 22: Position response: windup vs anti-windup")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day22_position.png"), dpi=200)
    plt.close()

    # Velocity
    plt.figure()
    plt.plot(t, x_w[:, 1], label="windup")
    plt.plot(t, x_aw[:, 1], label="anti-windup")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("velocity")
    plt.title("Day 22: Velocity response")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day22_velocity.png"), dpi=200)
    plt.close()

    # Control (saturated)
    u_min = data["params"]["u_min"]
    u_max = data["params"]["u_max"]

    plt.figure()
    plt.plot(t_u, data["u_sat_w"], label="u_sat (windup)")
    plt.plot(t_u, data["u_sat_aw"], label="u_sat (anti-windup)")
    plt.axhline(u_max, linestyle="--")
    plt.axhline(u_min, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("u (actuator command)")
    plt.title("Day 22 — Saturated control")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day22_control.png"), dpi=200)
    plt.close()

    # Integrator state
    plt.figure()
    plt.plot(t, data["I_w"], label="Integral state I (windup)")
    plt.plot(t, data["I_aw"], label="Integral state I (anti-windup)")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("time [s]")
    plt.ylabel("Integrator state")
    plt.title("Day 22: Integrator windup (look at divergence)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "day22_integrator.png"), dpi=200)
    plt.close()



# Main

def main():
    data = simulate_pid_windup_vs_antiwindup(
        dt=0.01,
        T=10.0,
        # Gains tuned to clearly show windup with tight actuator limits
        Kp=18.0,
        Ki=10.0,
        Kd=6.0,
        u_min=-1.0,
        u_max=1.0,
        Kaw=5.0,
    )

    base_dir = os.path.dirname(__file__) if "__file__" in globals() else os.getcwd()
    results_dir = os.path.join(base_dir, "results")
    make_plots(data, results_dir)

    p = data["params"]
    print("Day 22 control failure modes demo complete.")
    print(f"Saved plots to: {results_dir}")
    print(" - day22_position.png")
    print(" - day22_velocity.png")
    print(" - day22_control.png")
    print(" - day22_integrator.png")
    print("\nWhat to look for:")
    print(" - Windup case: integrator state grows big during saturation → overshoot/slow settle.")
    print(" - Anti-windup: integrator stays bounded → cleaner recovery.")
    print("\nParams:")
    print(f" dt={p['dt']}, T={p['T']}, Kp={p['Kp']}, Ki={p['Ki']}, Kd={p['Kd']}, Kaw={p['Kaw']}, u∈[{p['u_min']},{p['u_max']}]")
    print(f" x0={p['x0'].tolist()} → x_ref={p['x_ref'].tolist()}")


if __name__ == "__main__":
    main()