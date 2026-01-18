"""
Day 18: Controllability (LTI systems)

Checks controllability using:
1) Kalman controllability matrix rank test
2) PBH test (sanity check)
Also simulates reachable spread (2D only) for intuition.

Run:
    python controllability.py

Outputs:
    ./results/controllability_report.txt
    ./results/reachability_scatter.png (for 2D examples)
"""

import os
import numpy as np
import matplotlib.pyplot as plt



# Core controllability utilities

def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Kalman controllability matrix C = [B, AB, A^2B, ..., A^{n-1}B]."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def is_controllable_rank(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> tuple[bool, int]:
    """Returns (controllable?, rank(C))."""
    C = controllability_matrix(A, B)
    r = np.linalg.matrix_rank(C, tol=tol)
    return (r == A.shape[0], r)


def pbh_controllability(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> tuple[bool, list]:
    """
    PBH test for controllability:
    (A,B) controllable iff rank([λI - A, B]) = n for every eigenvalue λ of A.
    Returns (pass?, details list).
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    eigvals = np.linalg.eigvals(A)

    details = []
    ok = True
    for lam in eigvals:
        M = np.hstack([lam * np.eye(n) - A, B])  # becomes complex if lam is complex (fine)
        r = np.linalg.matrix_rank(M, tol=tol)
        details.append((lam, r))
        if r < n:
            ok = False
    return ok, details



# Reachability intuition (2D)

def simulate_reachability_2d(A, B, dt=0.02, T=2.0, u_samples=2000, u_bound=1.0, seed=0):
    """
    Simple discrete simulation for 2D systems:
        x_{k+1} = (I + A dt) x_k + (B dt) u_k
    Start from x0 = 0, random controls, collect final states.
    """
    rng = np.random.default_rng(seed)
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    n = A.shape[0]
    if n != 2:
        raise ValueError("simulate_reachability_2d expects a 2D system (n=2).")

    Ad = np.eye(n) + A * dt
    Bd = B * dt

    steps = int(T / dt)
    finals = []

    for _ in range(u_samples):
        x = np.zeros((n, 1))
        for _k in range(steps):
            m = B.shape[1]
            u = rng.uniform(-u_bound, u_bound, size=(m, 1))
            x = Ad @ x + Bd @ u
        finals.append(x.flatten())

    return np.array(finals)



# Pretty printing helpers

def fmt_matrix(M: np.ndarray, name: str) -> str:
    with np.printoptions(precision=4, suppress=True):
        return f"{name} =\n{M}\n"


def ensure_results_dir(path="./results"):
    os.makedirs(path, exist_ok=True)
    return path



# Main

def main():
    results_dir = ensure_results_dir("./results")
    report_path = os.path.join(results_dir, "controllability_report.txt")
    fig_path = os.path.join(results_dir, "reachability_scatter.png")

    # Example 1 (controllable): stable-ish 2nd order system
    A1 = np.array([[0.0, 1.0],
                   [-2.0, -0.7]])
    B1 = np.array([[0.0],
                   [1.0]])

    # Example 2 (NOT controllable): chain where input cannot influence x2 at all
    # x1_dot = x2
    # x2_dot = 0
    # u enters ONLY x1; but x2 never changes => cannot steer x2 => uncontrollable
    A2 = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
    B2 = np.array([[1.0],
                   [0.0]])

    systems = [
        ("System 1 (controllable)", A1, B1),
        ("System 2 (NOT controllable)", A2, B2),
    ]

    lines = []
    lines.append("Day 18: Controllability Report\n")
    lines.append("=" * 40 + "\n\n")

    for title, A, B in systems:
        lines.append(f"{title}\n")
        lines.append("-" * len(title) + "\n")
        lines.append(fmt_matrix(A, "A"))
        lines.append(fmt_matrix(B, "B"))

        C = controllability_matrix(A, B)
        controllable_rank, rC = is_controllable_rank(A, B)
        controllable_pbh, pbh_details = pbh_controllability(A, B)

        lines.append(fmt_matrix(C, "Controllability matrix C=[B AB ...]"))
        lines.append(f"Rank(C) = {rC} (n={A.shape[0]})\n")
        lines.append(f"Kalman rank test: {'CONTROLLABLE ✅' if controllable_rank else 'NOT controllable ❌'}\n")
        lines.append(f"PBH test:         {'CONTROLLABLE ✅' if controllable_pbh else 'NOT controllable ❌'}\n")
        lines.append("PBH eigenvalue ranks (rank([λI-A, B])):\n")
        for lam, rr in pbh_details:
            lam_str = f"{lam:.6g}" if np.isreal(lam) else f"{lam.real:.6g}+{lam.imag:.6g}j"
            lines.append(f"  λ={lam_str:>14}  rank={rr}\n")
        lines.append("\n")

    with open(report_path, "w", encoding="utf-8") as f:
        f.writelines(lines)

    print("Day 18: Controllability")
    print(f"Report saved to: {report_path}")

    # Reachability scatter (2D intuition)
    try:
        finals1 = simulate_reachability_2d(A1, B1, u_samples=2500, seed=1)
        finals2 = simulate_reachability_2d(A2, B2, u_samples=2500, seed=2)

        plt.figure(figsize=(7, 7))
        plt.scatter(finals1[:, 0], finals1[:, 1], s=6, alpha=0.35, label="System 1 final states")
        plt.scatter(finals2[:, 0], finals2[:, 1], s=6, alpha=0.35, label="System 2 final states")
        plt.axhline(0, linewidth=1)
        plt.axvline(0, linewidth=1)
        plt.title("Reachability intuition (random control sequences, final states)")
        plt.xlabel("x1")
        plt.ylabel("x2")
        plt.legend()
        plt.tight_layout()
        plt.savefig(fig_path, dpi=200)
        plt.close()

        print(f"Reachability scatter saved to: {fig_path}")
    except Exception as e:
        print(f"(Skipped reachability plot) Reason: {e}")


if __name__ == "__main__":
    main()