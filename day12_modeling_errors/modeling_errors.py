import os
import numpy as np
import matplotlib.pyplot as plt


# Helpers

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def ctrb(A, B):
    """Controllability matrix."""
    n = A.shape[0]
    C = B
    AB = B
    for _ in range(1, n):
        AB = A @ AB
        C = np.hstack((C, AB))
    return C

def place_poles_2x2(A, B, desired_poles):
    """
    Pole placement for 2x2 single-input using Ackermann's formula.
    Works for controllable (A,B) with B shape (2,1).
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float).reshape(2, 1)
    p1, p2 = desired_poles

    # Desired characteristic polynomial: (s - p1)(s - p2) = s^2 - (p1+p2)s + p1*p2
    a1 = -(p1 + p2)   # coefficient of s
    a0 = (p1 * p2)    # constant term

    # phi(A) = A^2 + a1*A + a0*I  (since desired poly: s^2 + a1*s + a0)
    I = np.eye(2)
    phiA = (A @ A) + a1 * A + a0 * I

    Cc = ctrb(A, B)
    if np.linalg.matrix_rank(Cc) < 2:
        raise ValueError("System not controllable for given (A,B). Can't place poles.")

    # Ackermann: K = e_n^T * Cc^{-1} * phi(A)
    eT = np.array([[0.0, 1.0]])  # [0 1]
    K = eT @ np.linalg.inv(Cc) @ phiA
    return K  # shape (1,2)

def simulate_closed_loop(A, B, K, x0, t_end=8.0, dt=0.001, u_limit=None):
    """
    Simulate xdot = (A - B K) x, with optional actuator saturation.
    Uses simple Euler integration (enough for this demo).
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float).reshape(2, 1)
    K = np.array(K, dtype=float).reshape(1, 2)

    N = int(t_end / dt) + 1
    t = np.linspace(0, t_end, N)
    x = np.zeros((N, 2))
    u = np.zeros(N)

    x[0] = np.array(x0, dtype=float).reshape(2,)
    for k in range(N - 1):
        # nominal control
        u_k = float(-K @ x[k].reshape(2, 1))

        # optional saturation
        if u_limit is not None:
            u_k = np.clip(u_k, -abs(u_limit), abs(u_limit))

        u[k] = u_k
        xdot = (A @ x[k].reshape(2, 1)) + (B * u_k)
        x[k + 1] = x[k] + (dt * xdot.reshape(2,))

    u[-1] = float(-K @ x[-1].reshape(2, 1))
    if u_limit is not None:
        u[-1] = float(np.clip(u[-1], -abs(u_limit), abs(u_limit)))

    return t, x, u

def msd_state_space(m=1.0, c=0.2, k=5.0):
    """
    Mass-spring-damper:
        m xddot + c xdot + k x = u
    state: [x, xdot]
    """
    A = np.array([[0.0, 1.0],
                  [-k/m, -c/m]])
    B = np.array([[0.0],
                  [1.0/m]])
    return A, B



def main():
    ensure_dir("results")

    # Nominal parameters
    m0, c0, k0 = 1.0, 0.25, 5.0
    A0, B0 = msd_state_space(m=m0, c=c0, k=k0)

    # Design controller on NOMINAL model
    desired_poles = [-2.0, -3.0]  # stable, moderately fast
    K = place_poles_2x2(A0, B0, desired_poles)

    # Initial condition: position offset, zero velocity
    x0 = [1.0, 0.0]

    # Sim settings
    t_end = 8.0
    dt = 0.001
    u_limit = 10.0  # set None to disable saturation

    # Perturbations (modeling errors)
    cases = [
        ("nominal", dict(m=m0, c=c0, k=k0)),
        ("mass +20%", dict(m=1.2*m0, c=c0, k=k0)),
        ("damping -40%", dict(m=m0, c=0.6*c0, k=k0)),
        ("stiffness +30%", dict(m=m0, c=c0, k=1.3*k0)),
        ("all-wrong (m+20,c-40,k+30)", dict(m=1.2*m0, c=0.6*c0, k=1.3*k0)),
    ]

    # Run sims
    results = {}
    for name, params in cases:
        A, B = msd_state_space(**params)
        t, x, u = simulate_closed_loop(A, B, K, x0, t_end=t_end, dt=dt, u_limit=u_limit)
        results[name] = (t, x, u, params)


    # Plot: position response

    plt.figure()
    for name, (t, x, u, params) in results.items():
        plt.plot(t, x[:, 0], label=name)
    plt.xlabel("Time (s)")
    plt.ylabel("Position x (m)")
    plt.title("Closed-loop position: nominal K, real plant varies (modeling error)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/position_responses.png", dpi=200)


    # Plot: control effort

    plt.figure()
    for name, (t, x, u, params) in results.items():
        plt.plot(t, u, label=name)
    plt.xlabel("Time (s)")
    plt.ylabel("Control input u (N)")
    title = "Control effort (with saturation)" if u_limit is not None else "Control effort"
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/control_effort.png", dpi=200)

    # Quick console summary
    print("=== Day 12: Modeling Errors Experiment ===")
    print(f"Nominal params: m={m0}, c={c0}, k={k0}")
    print(f"Designed poles on nominal: {desired_poles}")
    print(f"State feedback gain K = {K}")
    if u_limit is not None:
        print(f"Actuator saturation: ±{u_limit}")

    # crude metrics: final abs position + max overshoot
    for name, (t, x, u, params) in results.items():
        pos = x[:, 0]
        final_abs = abs(pos[-1])
        overshoot = np.max(pos)  # since initial is +1, overshoot here means peak above 1
        u_peak = np.max(np.abs(u))
        print(f"\n[{name}] params={params}")
        print(f"  final |x| = {final_abs:.4f}")
        print(f"  peak x = {overshoot:.4f}")
        print(f"  peak |u| = {u_peak:.4f}")

    print("\nSaved plots:")
    print(" - results/position_responses.png")
    print(" - results/control_effort.png")

if __name__ == "__main__":
    main()