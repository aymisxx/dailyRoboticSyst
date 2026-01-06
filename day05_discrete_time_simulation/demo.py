import numpy as np
import matplotlib.pyplot as plt

# System definition

a = 1.0

def dynamics(x):
    """Continuous-time dynamics: x_dot = -a x"""
    return -a * x


# Forward Euler simulator

def simulate_euler(x0, dt, T):
    """
    Simulate x_dot = -a x using forward Euler integration.

    Parameters
    ----------
    x0 : float
        Initial state
    dt : float
        Time step
    T : float
        Total simulation time

    Returns
    -------
    t : ndarray
        Time vector
    x : ndarray
        State trajectory
    """
    N = int(np.floor(T / dt)) + 1
    t = np.linspace(0.0, dt * (N - 1), N)
    x = np.zeros(N)
    x[0] = x0

    for k in range(N - 1):
        x[k + 1] = x[k] + dt * dynamics(x[k])

    return t, x


# Main experiment

if __name__ == "__main__":
    x0 = 1.0
    T = 5.0
    dt_values = [0.1, 1.5, 2.1]

    plt.figure()
    for dt in dt_values:
        t, x = simulate_euler(x0, dt, T)
        plt.plot(t, x, label=f"dt = {dt}")

    plt.xlabel("Time (s)")
    plt.ylabel("State x")
    plt.title("Effect of Time Step on Euler Simulation")
    plt.legend()
    plt.grid(True)
    plt.show()