"""
Day 20: Constraints & Actuator Saturation (dailyRoboticSyst)

Experiment:
1) PID without saturation (ideal actuator)
2) PID with saturation (windup happens)
3) PID with saturation + anti-windup (back-calculation)

Plant: Mass-Spring-Damper (MSD)
State: x = [position, velocity]
Dynamics:
    x_dot = v
    v_dot = (1/m) * (u - c*v - k*x)

Control objective: Track reference position r(t) (default step to 1.0)
"""

from __future__ import annotations

import os
import json
from dataclasses import dataclass
from typing import Dict

import numpy as np
import matplotlib.pyplot as plt



# Utilities


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)

def clamp(u: float, umin: float, umax: float) -> float:
    return max(umin, min(umax, u))

def trapz_energy(u: np.ndarray, t: np.ndarray) -> float:
    # control effort energy ~ ∫ u^2 dt
    return float(np.trapezoid(u**2, t))

def settling_time(y: np.ndarray, t: np.ndarray, y_final: float, tol: float = 0.02) -> float:
    """
    First time after which response stays within tol band around final value.
    tol=0.02 means +/-2% band.
    """
    if len(t) < 2:
        return float("nan")

    band = tol * max(1e-9, abs(y_final))
    lo, hi = y_final - band, y_final + band

    outside = np.where((y < lo) | (y > hi))[0]
    if outside.size == 0:
        return float(t[0])

    last_out = outside[-1]
    if last_out >= len(t) - 1:
        return float("nan")

    return float(t[last_out + 1])

def overshoot(y: np.ndarray, y_final: float) -> float:
    if abs(y_final) < 1e-9:
        return float(np.max(np.abs(y)))
    return float((np.max(y) - y_final) / abs(y_final) * 100.0)

def rise_time(y: np.ndarray, t: np.ndarray, y_final: float, lo: float = 0.1, hi: float = 0.9) -> float:
    # assumes positive final step; fine for this experiment
    if abs(y_final) < 1e-9:
        return float("nan")
    ylo, yhi = lo * y_final, hi * y_final

    idx_lo = np.where(y >= ylo)[0]
    idx_hi = np.where(y >= yhi)[0]
    if idx_lo.size == 0 or idx_hi.size == 0:
        return float("nan")
    return float(t[idx_hi[0]] - t[idx_lo[0]])



# Models


@dataclass
class MSDParams:
    m: float = 1.0
    c: float = 0.8
    k: float = 4.0

@dataclass
class PIDParams:
    kp: float = 18.0
    ki: float = 8.0
    kd: float = 3.0
    # Back-calculation gain ~ 1/Tt (bigger => stronger anti-windup correction)
    kaw: float = 15.0

@dataclass
class SimParams:
    dt: float = 0.001
    T: float = 6.0
    x0: float = 0.0
    v0: float = 0.0
    # Actuator limits
    umin: float = -6.0
    umax: float = 6.0
    # Reference (step)
    r_step: float = 1.0
    r_time: float = 0.2  # step at 0.2s to avoid initial derivative spike


def reference(t: float, r_step: float, r_time: float) -> float:
    return r_step if t >= r_time else 0.0



# Simulation core


def simulate_case(
    msd: MSDParams,
    pid: PIDParams,
    sim: SimParams,
    use_saturation: bool,
    use_anti_windup: bool,
) -> Dict[str, np.ndarray]:
    """
    Discrete-time Euler integration (simple + transparent).
    """
    n = int(sim.T / sim.dt) + 1
    t = np.linspace(0.0, sim.T, n)

    x = np.zeros(n)
    v = np.zeros(n)
    r = np.zeros(n)
    e = np.zeros(n)

    u_unsat = np.zeros(n)   # controller request
    u_sat = np.zeros(n)     # actuator output

    I = 0.0
    e_prev = 0.0

    x[0] = sim.x0
    v[0] = sim.v0

    for i in range(n - 1):
        ti = t[i]
        ri = reference(ti, sim.r_step, sim.r_time)
        r[i] = ri

        ei = ri - x[i]
        e[i] = ei

        de = (ei - e_prev) / sim.dt if i > 0 else 0.0

        # integrate error (candidate)
        I_candidate = I + ei * sim.dt

        # unsaturated controller command
        u_cmd = pid.kp * ei + pid.ki * I_candidate + pid.kd * de
        u_unsat[i] = u_cmd

        # actuator saturation
        if use_saturation:
            u_real = clamp(u_cmd, sim.umin, sim.umax)
        else:
            u_real = u_cmd

        # anti-windup: back-calculate into integrator state
        # I_dot = e + (kaw/ki)*(u_sat - u_unsat)
        # (the /ki maps the correction into the integrator STATE, not its output Ki*I)
        if use_anti_windup and use_saturation and pid.ki > 1e-12:
            I = I_candidate + (pid.kaw / pid.ki) * (u_real - u_cmd) * sim.dt
        else:
            I = I_candidate

        u_sat[i] = u_real

        # plant update (Euler)
        a = (u_real - msd.c * v[i] - msd.k * x[i]) / msd.m
        x[i + 1] = x[i] + v[i] * sim.dt
        v[i + 1] = v[i] + a * sim.dt

        e_prev = ei

    # final samples (for plotting convenience)
    r[-1] = reference(t[-1], sim.r_step, sim.r_time)
    e[-1] = r[-1] - x[-1]
    u_unsat[-1] = u_unsat[-2]
    u_sat[-1] = u_sat[-2]

    return {"t": t, "x": x, "v": v, "r": r, "e": e, "u_unsat": u_unsat, "u": u_sat}


def compute_metrics(data: Dict[str, np.ndarray], sim: SimParams) -> Dict[str, float]:
    t = data["t"]
    x = data["x"]
    r = data["r"]
    u = data["u"]

    y_final = float(r[-1])
    eps = 1e-9
    sat_mask = (u >= sim.umax - eps) | (u <= sim.umin + eps)

    return {
        "peak_position": float(np.max(x)),
        "peak_abs_error": float(np.max(np.abs(r - x))),
        "overshoot_percent": overshoot(x, y_final),
        "rise_time_s": rise_time(x, t, y_final),
        "settling_time_s": settling_time(x, t, y_final, tol=0.02),
        "control_energy_int_u2_dt": trapz_energy(u, t),
        "u_max": float(np.max(u)),
        "u_min": float(np.min(u)),
        "time_at_saturation_frac": float(np.mean(sat_mask)),
    }



# Plotting


def make_plots(all_data: Dict[str, Dict[str, np.ndarray]], outdir: str, sim: SimParams) -> None:
    ensure_dir(outdir)

    any_case = next(iter(all_data.values()))

    # 1) Position tracking
    plt.figure(figsize=(10, 5))
    for name, d in all_data.items():
        plt.plot(d["t"], d["x"], label=name)
    plt.plot(any_case["t"], any_case["r"], "--", label="reference")
    plt.title("Position tracking: effect of saturation + anti-windup")
    plt.xlabel("Time (s)")
    plt.ylabel("Position x (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "position_tracking.png"), dpi=180)
    plt.close()

    # 2) Control effort (real u)
    plt.figure(figsize=(10, 5))
    for name, d in all_data.items():
        plt.plot(d["t"], d["u"], label=f"{name}: u")
    plt.axhline(sim.umax, linestyle="--")
    plt.axhline(sim.umin, linestyle="--")
    plt.title("Control input u (with actuator limits)")
    plt.xlabel("Time (s)")
    plt.ylabel("u")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "control_u_real.png"), dpi=180)
    plt.close()

    # 3) u_cmd vs u_real for saturated cases
    plt.figure(figsize=(10, 5))
    for name, d in all_data.items():
        if "SAT" in name:
            plt.plot(d["t"], d["u_unsat"], label=f"{name}: u_cmd (unsat)")
            plt.plot(d["t"], d["u"], label=f"{name}: u_real (sat)")
    plt.axhline(sim.umax, linestyle="--")
    plt.axhline(sim.umin, linestyle="--")
    plt.title("Windup indicator: u_cmd grows while u_real is clamped")
    plt.xlabel("Time (s)")
    plt.ylabel("u")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "u_cmd_vs_u_real.png"), dpi=180)
    plt.close()

    # 4) Error curves
    plt.figure(figsize=(10, 5))
    for name, d in all_data.items():
        plt.plot(d["t"], d["e"], label=name)
    plt.title("Tracking error e = r - x")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "tracking_error.png"), dpi=180)
    plt.close()



# Main


def main() -> None:
    msd = MSDParams(m=1.0, c=0.8, k=4.0)
    pid = PIDParams(kp=18.0, ki=8.0, kd=3.0, kaw=15.0)
    sim = SimParams(dt=0.001, T=6.0, x0=0.0, v0=0.0, umin=-6.0, umax=6.0, r_step=1.0, r_time=0.2)

    cases = {
        "IDEAL (no sat)": dict(use_saturation=False, use_anti_windup=False),
        "SAT (windup)": dict(use_saturation=True, use_anti_windup=False),
        "SAT + AW (fixed)": dict(use_saturation=True, use_anti_windup=True),
    }

    all_data: Dict[str, Dict[str, np.ndarray]] = {}
    all_metrics: Dict[str, Dict[str, float]] = {}

    print("\nDay 20: Constraints & Actuator Saturation\n")
    print(f"MSD: m={msd.m}, c={msd.c}, k={msd.k}")
    print(f"PID: kp={pid.kp}, ki={pid.ki}, kd={pid.kd}, kaw={pid.kaw}")
    print(f"Actuator limits: [{sim.umin}, {sim.umax}]\n")

    for name, flags in cases.items():
        d = simulate_case(msd, pid, sim, flags["use_saturation"], flags["use_anti_windup"])
        all_data[name] = d
        all_metrics[name] = compute_metrics(d, sim)

    for name, met in all_metrics.items():
        print(f"{name}")
        for k, v in met.items():
            print(f"{k:28s}: {v}")
        print()

    outdir = os.path.join(os.path.dirname(__file__), "results")
    make_plots(all_data, outdir, sim)

    summary = {
        "msd_params": msd.__dict__,
        "pid_params": pid.__dict__,
        "sim_params": sim.__dict__,
        "metrics": all_metrics,
    }
    ensure_dir(outdir)
    with open(os.path.join(outdir, "run_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"Saved results to: {outdir}")
    print("Generated:")
    print(" - position_tracking.png")
    print(" - control_u_real.png")
    print(" - u_cmd_vs_u_real.png")
    print(" - tracking_error.png")
    print(" - run_summary.json\n")


if __name__ == "__main__":
    main()