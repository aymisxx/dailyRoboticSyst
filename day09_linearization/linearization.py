"""
Day 09: Linearization (Nonlinear Pendulum) [Corrected + Robust]

Fixes vs previous version:
1) Uses true equilibria for ALL operating points by choosing u_eq = (g/l)*sin(theta_eq).
2) Compares *deviations* using angle wrapping (wrap_to_pi), so near_inverted doesn't explode visually.
3) Uses smaller initial perturbation for unstable inverted equilibrium (pi), to stay local briefly.
4) Linear comparison is clipped once the linearized deviation leaves a local neighborhood (optional but enabled).

Run:
    python day09_linearization_pendulum.py
Outputs:
    results/*.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt



# Utilities

def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def wrap_to_pi(angle):
    """Wrap any angle array/scalar to (-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rk4_step(f, x, u, dt):
    """One RK4 step for xdot = f(x,u)."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def jacobian_x_central(f, x, u, eps=1e-6):
    """Numerical Jacobian wrt x using central differences."""
    x = np.asarray(x, dtype=float)
    n = x.size
    fx = f(x, u)
    m = fx.size
    J = np.zeros((m, n))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        fp = f(x + dx, u)
        fm = f(x - dx, u)
        J[:, i] = (fp - fm) / (2.0 * eps)
    return J


def jacobian_u_central(f, x, u, eps=1e-6):
    """Numerical Jacobian wrt u using central differences."""
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)
    p = u.size
    fx = f(x, u)
    m = fx.size
    J = np.zeros((m, p))
    for i in range(p):
        du = np.zeros(p)
        du[i] = eps
        fp = f(x, u + du)
        fm = f(x, u - du)
        J[:, i] = (fp - fm) / (2.0 * eps)
    return J



# System: Nonlinear Pendulum

def pendulum_f_factory(g=9.81, l=1.0, b=0.05, include_input=True):
    """
    Damped pendulum with torque input.
    State: x = [theta, omega]
    Input: u = [tau] if include_input else ignored

    Dynamics:
        theta_dot = omega
        omega_dot = -(g/l) * sin(theta) - b*omega + tau
    """
    def f(x, u):
        theta, omega = x
        tau = float(u[0]) if include_input else 0.0
        return np.array([omega, -(g / l) * np.sin(theta) - b * omega + tau], dtype=float)

    return f


def pendulum_jacobians_analytic(theta_eq, g=9.81, l=1.0, b=0.05, include_input=True):
    """
    Analytic Jacobians at (theta_eq, omega_eq=0).
    A = df/dx, B = df/du
    """
    A = np.array(
        [[0.0, 1.0],
         [-(g / l) * np.cos(theta_eq), -b]],
        dtype=float,
    )
    B = np.array([[0.0], [1.0]], dtype=float) if include_input else np.zeros((2, 1), dtype=float)
    return A, B


def equilibrium_torque_for_theta(theta_eq, g=9.81, l=1.0, include_input=True):
    """
    Choose u_eq to make x_eq=[theta_eq,0] an equilibrium:
        0 = -(g/l) sin(theta_eq) + u_eq  => u_eq = (g/l) sin(theta_eq)
    """
    if not include_input:
        return 0.0
    return (g / l) * np.sin(theta_eq)


def equilibrium_residual(f, x_eq, u_eq):
    """||f(x_eq,u_eq)|| (should be ~0 for equilibrium)."""
    return float(np.linalg.norm(f(x_eq, u_eq)))



# Linearized deviation model

def linearized_deviation_step(A, B, dx, du, dt):
    """
    Deviation dynamics:
        d(dx)/dt = A dx + B du
    integrated with RK4.
    """
    def g(dx_, du_):
        return A @ dx_ + B @ du_

    return rk4_step(g, dx, du, dt)



# Simulation harness

def simulate_compare(
    f,
    A,
    B,
    x_eq,
    u_eq,
    x0,
    u_of_t,
    t,
    local_clip_rad=0.35,
):
    """
    Simulate:
    - nonlinear: xdot = f(x,u)
    - linearized deviation: d(dx)/dt = A dx + B du
      where dx = x - x_eq, du = u - u_eq

    local_clip_rad:
      stops linearized integration once |wrap(theta_lin - theta_eq)| exceeds this threshold
      (because linearization is no longer valid globally).
    """
    dt = float(t[1] - t[0])

    x_nl = np.zeros((len(t), 2))
    dx_lin = np.zeros((len(t), 2))
    u_hist = np.zeros((len(t), 1))

    x_nl[0] = x0
    dx_lin[0] = (x0 - x_eq)

    stop_k = len(t) - 1  # when we stop trusting linear model

    for k in range(len(t) - 1):
        uk = np.array([u_of_t(t[k])], dtype=float)
        u_hist[k] = uk

        # Nonlinear step always runs full horizon
        x_nl[k + 1] = rk4_step(f, x_nl[k], uk, dt)

        # Linearized deviation step (only while local)
        if k < stop_k:
            du = uk - u_eq
            dx_lin[k + 1] = linearized_deviation_step(A, B, dx_lin[k], du, dt)

            theta_lin = dx_lin[k + 1, 0] + x_eq[0]
            if abs(wrap_to_pi(theta_lin - x_eq[0])) > local_clip_rad:
                stop_k = k + 1  # freeze after this index
        else:
            dx_lin[k + 1] = dx_lin[k]  # freeze linear state after leaving local region

    u_hist[-1] = np.array([u_of_t(t[-1])], dtype=float)
    x_lin = dx_lin + x_eq

    return x_nl, x_lin, u_hist, stop_k



# Main

def main():
    # Parameters
    g = 9.81
    l = 1.0
    b = 0.05
    include_input = True

    # Time
    dt = 0.002
    t_end = 8.0
    t = np.arange(0.0, t_end + dt, dt)

    # Build system
    f = pendulum_f_factory(g=g, l=l, b=b, include_input=include_input)

    # Operating points
    operating_points = [
        ("upright_small_angle", 0.0),
        ("near_inverted", np.pi),
        ("tilted_0p6rad", 0.6),
    ]

    # Small excitation around equilibrium input
    def u_delta_of_t(tt):
        pulse = 0.8 if 1.0 <= tt <= 1.3 else 0.0
        sinus = 0.2 * np.sin(2.0 * tt)
        return pulse + sinus

    out_dir = "results"
    ensure_dir(out_dir)

    print("Day 09: Linearization (Nonlinear Pendulum) [Corrected + Robust]")
    print(f"Params: g={g}, l={l}, b={b}, dt={dt}, t_end={t_end}")
    print(f"Results will be saved to: {out_dir}/\n")

    for tag, theta_eq in operating_points:
        x_eq = np.array([theta_eq, 0.0], dtype=float)

        u_eq_val = equilibrium_torque_for_theta(theta_eq, g=g, l=l, include_input=include_input)
        u_eq = np.array([u_eq_val], dtype=float)

        res = equilibrium_residual(f, x_eq, u_eq)

        # Jacobians
        A_an, B_an = pendulum_jacobians_analytic(theta_eq, g=g, l=l, b=b, include_input=include_input)
        A_num = jacobian_x_central(f, x_eq, u_eq, eps=1e-6)
        B_num = jacobian_u_central(f, x_eq, u_eq, eps=1e-6)

        eigs = np.linalg.eigvals(A_an)

        print(f"Operating point: {tag}")
        print(f"x_eq = [theta={theta_eq:.6f}, omega=0]")
        print(f"u_eq = {u_eq_val:.6f}")
        print(f"Equilibrium residual ||f(x_eq,u_eq)|| = {res:.3e}")
        print("A (analytic):\n", A_an)
        print("B (analytic):\n", B_an)
        print("eig(A):", np.array2string(eigs, precision=4))
        print("||A_an - A_num||_F =", np.linalg.norm(A_an - A_num))
        print("||B_an - B_num||_F =", np.linalg.norm(B_an - B_num))
        print()

        # Initial perturbation (keep inverted VERY small; others can be larger)
        if np.isclose(theta_eq, np.pi):
            dtheta0 = 0.01
        else:
            dtheta0 = 0.15

        x0 = x_eq + np.array([dtheta0, 0.0], dtype=float)

        # Actual input u(t) = u_eq + delta_u(t)
        def u_of_t(tt):
            return float(u_eq_val + u_delta_of_t(tt))

        # Simulate
        x_nl, x_lin, u_hist, stop_k = simulate_compare(
            f=f,
            A=A_an,
            B=B_an,
            x_eq=x_eq,
            u_eq=u_eq,
            x0=x0,
            u_of_t=u_of_t,
            t=t,
            local_clip_rad=0.35,
        )

        # Wrapped deviations for fair comparison
        dtheta_nl = wrap_to_pi(x_nl[:, 0] - x_eq[0])
        dtheta_lin = wrap_to_pi(x_lin[:, 0] - x_eq[0])

        domega_nl = x_nl[:, 1] - x_eq[1]
        domega_lin = x_lin[:, 1] - x_eq[1]

        # Plots: deviations (this is the correct lens for linearization)
        fig1 = plt.figure()
        plt.plot(t, dtheta_nl, label="δtheta (nonlinear)")
        plt.plot(t, dtheta_lin, label="δtheta (linearized)")
        plt.xlabel("time [s]")
        plt.ylabel("δtheta [rad]")
        plt.title(f"Pendulum δθ: Nonlinear vs Linearized ({tag})")
        plt.legend()
        plt.grid(True)
        path1 = os.path.join(out_dir, f"{tag}_dtheta_compare.png")
        plt.savefig(path1, dpi=200, bbox_inches="tight")
        plt.close(fig1)

        fig2 = plt.figure()
        plt.plot(t, domega_nl, label="δomega (nonlinear)")
        plt.plot(t, domega_lin, label="δomega (linearized)")
        plt.xlabel("time [s]")
        plt.ylabel("δomega [rad/s]")
        plt.title(f"Pendulum δω: Nonlinear vs Linearized ({tag})")
        plt.legend()
        plt.grid(True)
        path2 = os.path.join(out_dir, f"{tag}_domega_compare.png")
        plt.savefig(path2, dpi=200, bbox_inches="tight")
        plt.close(fig2)

        fig3 = plt.figure()
        plt.plot(t, u_hist[:, 0], label="u(t)")
        plt.xlabel("time [s]")
        plt.ylabel("torque input [arb]")
        plt.title(f"Input u(t) ({tag})")
        plt.legend()
        plt.grid(True)
        path3 = os.path.join(out_dir, f"{tag}_input.png")
        plt.savefig(path3, dpi=200, bbox_inches="tight")
        plt.close(fig3)

        fig4 = plt.figure()
        err_dtheta = dtheta_nl - dtheta_lin
        plt.plot(t, err_dtheta, label="δtheta error (nonlinear - linearized)")
        plt.xlabel("time [s]")
        plt.ylabel("error [rad]")
        plt.title(f"Linearization error in δθ ({tag})")
        plt.legend()
        plt.grid(True)
        path4 = os.path.join(out_dir, f"{tag}_dtheta_error.png")
        plt.savefig(path4, dpi=200, bbox_inches="tight")
        plt.close(fig4)

        # Optional: show where we stopped trusting the linear model
        if stop_k < len(t) - 1:
            print(f"Note: Linear model clipped at t={t[stop_k]:.3f}s (left local region).")

        print(f"Saved plots for {tag}:")
        print(f" - {path1}")
        print(f" - {path2}")
        print(f" - {path3}")
        print(f" - {path4}\n")

    print("Done.")


if __name__ == "__main__":
    main()