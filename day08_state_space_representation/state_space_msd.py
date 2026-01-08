# Mass–Spring–Damper in State-Space: x_dot = A x + B u
# States: x = [position, velocity]
# Output: y = position

import os
import numpy as np
import matplotlib.pyplot as plt


def simulate_msd_state_space(
    m: float = 1.0,
    c: float = 0.4,
    k: float = 4.0,
    x0: np.ndarray | None = None,
    t_end: float = 10.0,
    dt: float = 0.001,
    step_u: float = 1.0,
    step_time: float = 1.0,
    mode: str = "step",  # "zero" or "step"
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulates a mass-spring-damper system using forward Euler integration.

    Continuous dynamics:
        m x_ddot + c x_dot + k x = u

    State-space with x = [x, x_dot]:
        x_dot = A x + B u

    Returns:
        t: time array (N,)
        X: state trajectory (N, 2) columns -> [position, velocity]
        U: input trajectory (N,)
    """
    if x0 is None:
        x0 = np.array([1.0, 0.0], dtype=float)  # initial displacement

    # A, B for continuous-time model
    A = np.array([[0.0, 1.0],
                  [-k / m, -c / m]], dtype=float)
    B = np.array([[0.0],
                  [1.0 / m]], dtype=float)

    # time grid
    t = np.arange(0.0, t_end + dt, dt)
    n = t.size

    # input signal
    U = np.zeros(n, dtype=float)
    if mode == "step":
        U[t >= step_time] = step_u
    elif mode == "zero":
        pass
    else:
        raise ValueError("mode must be 'zero' or 'step'")

    # simulate
    X = np.zeros((n, 2), dtype=float)
    X[0] = x0

    for i in range(n - 1):
        u = U[i]
        xdot = A @ X[i] + (B.flatten() * u)  # (2,) + (2,)
        X[i + 1] = X[i] + dt * xdot

    return t, X, U


def main():
    # Params 
    m, c, k = 1.0, 0.4, 4.0
    x0 = np.array([1.0, 0.0])  # initial position=1, velocity=0
    t_end, dt = 10.0, 0.001

    # Case 1: zero input
    t0, X0, U0 = simulate_msd_state_space(
        m=m, c=c, k=k, x0=x0, t_end=t_end, dt=dt,
        mode="zero"
    )

    # Case 2: step input
    t1, X1, U1 = simulate_msd_state_space(
        m=m, c=c, k=k, x0=x0, t_end=t_end, dt=dt,
        step_u=1.0, step_time=1.0,
        mode="step"
    )

    # plots
    os.makedirs("results", exist_ok=True)

    # Plot states: position and velocity (zero input)
    plt.figure()
    plt.plot(t0, X0[:, 0], label="position x(t)")
    plt.plot(t0, X0[:, 1], label="velocity x_dot(t)")
    plt.xlabel("time (s)")
    plt.ylabel("state value")
    plt.title("Mass–Spring–Damper (State-Space): Zero Input")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/zero_input_states.png", dpi=200)

    # Plot states: position and velocity (step input)
    plt.figure()
    plt.plot(t1, X1[:, 0], label="position x(t)")
    plt.plot(t1, X1[:, 1], label="velocity x_dot(t)")
    plt.xlabel("time (s)")
    plt.ylabel("state value")
    plt.title("Mass–Spring–Damper (State-Space): Step Input (u=1 at t=1s)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/step_input_states.png", dpi=200)

    # Plot input for step case (optional but nice)
    plt.figure()
    plt.plot(t1, U1)
    plt.xlabel("time (s)")
    plt.ylabel("u(t)")
    plt.title("Input Signal: Step Input")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/step_input_u.png", dpi=200)

    # quick console summary (useful for README later)
    print("Day 08: State-Space MSD")
    print(f"m={m}, c={c}, k={k}, dt={dt}, t_end={t_end}")
    print(f"Initial state x0=[{x0[0]:.3f}, {x0[1]:.3f}]")
    print("--- Zero input final state ---")
    print(f"x(T)={X0[-1,0]:.6f}, x_dot(T)={X0[-1,1]:.6f}")
    print("--- Step input final state ---")
    print(f"x(T)={X1[-1,0]:.6f}, x_dot(T)={X1[-1,1]:.6f}")
    print("Saved plots in: results/")
    print(" - results/zero_input_states.png")
    print(" - results/step_input_states.png")
    print(" - results/step_input_u.png")


if __name__ == "__main__":
    main()