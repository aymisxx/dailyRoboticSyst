import os
import numpy as np
import matplotlib.pyplot as plt


def simulate_linear_system(A: np.ndarray, x0: np.ndarray, dt: float, T: float):
    """
    Simulate x_dot = A x using forward Euler:
        x_{k+1} = x_k + dt * (A x_k)

    Returns:
        t: (N,)
        X: (N, n) where X[k] = x(t_k)
    """
    n = A.shape[0]
    assert A.shape == (n, n), "A must be square"
    assert x0.shape == (n,), "x0 must be shape (n,)"
    assert dt > 0, "dt must be > 0"
    assert T >= 0, "T must be >= 0"

    N = int(np.floor(T / dt)) + 1
    t = np.linspace(0.0, dt * (N - 1), N)

    X = np.zeros((N, n), dtype=float)
    X[0] = x0.astype(float)

    for k in range(N - 1):
        x = X[k]
        x_dot = A @ x
        X[k + 1] = x + dt * x_dot

    return t, X


def ensure_results_dir(path="results"):
    os.makedirs(path, exist_ok=True)
    return path


def main():
    # =========================
    # 1) Define a 2-state system
    # =========================
    # A = [[0, 1],
    #      [-w^2, -2*zeta*w]]
    w = 2.0
    zeta = 0.2

    A = np.array([[0.0, 1.0],
                  [-(w ** 2), -2.0 * zeta * w]])

    print("\nSystem matrix A:\n", A)

    # =========================
    # 2) Eigen-analysis
    # =========================
    eigvals, eigvecs = np.linalg.eig(A)
    print("\nEigenvalues (lambda):", eigvals)
    print("Real parts:", np.real(eigvals))
    print("Imag parts:", np.imag(eigvals))

    if np.all(np.real(eigvals) < 0):
        stability = "STABLE (all Re(lambda) < 0)"
    elif np.any(np.real(eigvals) > 0):
        stability = "UNSTABLE (some Re(lambda) > 0)"
    else:
        stability = "MARGINALLY STABLE / BORDERLINE (some Re(lambda) = 0)"
    print("\nStability verdict:", stability)

    # =========================
    # 3) Simulate
    # =========================
    x0 = np.array([1.0, 0.0])   # position=1, velocity=0
    dt = 0.01
    T = 10.0

    t, X = simulate_linear_system(A, x0, dt, T)
    p = X[:, 0]
    v = X[:, 1]

    # =========================
    # 4) Plot and save results
    # =========================
    outdir = ensure_results_dir("results")

    # (a) State time series
    fig = plt.figure()
    plt.plot(t, p, label="position p(t)")
    plt.plot(t, v, label="velocity v(t)")
    plt.xlabel("time (s)")
    plt.ylabel("state value")
    plt.title("State vs Time for x_dot = A x")
    plt.legend()
    plt.grid(True)
    state_path = os.path.join(outdir, "state_time_series.png")
    plt.savefig(state_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # (b) Phase portrait (v vs p)
    fig = plt.figure()
    plt.plot(p, v)
    plt.xlabel("position p")
    plt.ylabel("velocity v")
    plt.title("Phase Portrait (v vs p)")
    plt.grid(True)
    phase_path = os.path.join(outdir, "phase_portrait.png")
    plt.savefig(phase_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # (c) Eigenvectors visualization (direction intuition)
    fig = plt.figure()
    plt.plot(p, v, alpha=0.5, label="trajectory")

    # NOTE: eigenvectors may be complex for oscillatory systems.
    # For a simple visual cue, we plot the real part direction.
    for i in range(eigvecs.shape[1]):
        vec = np.real(eigvecs[:, i])
        norm = np.linalg.norm(vec) + 1e-12
        vec = vec / norm
        plt.arrow(0, 0, vec[0], vec[1], head_width=0.05, length_includes_head=True)
        plt.text(vec[0] * 1.1, vec[1] * 1.1, f"v{i+1}", fontsize=10)

    plt.xlabel("position p")
    plt.ylabel("velocity v")
    plt.title("Trajectory + Eigenvector Directions")
    plt.grid(True)
    plt.legend()
    eig_path = os.path.join(outdir, "eigenvectors.png")
    plt.savefig(eig_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("\nSaved plots:")
    print(" -", state_path)
    print(" -", phase_path)
    print(" -", eig_path)

    plt.show()


if __name__ == "__main__":
    main()