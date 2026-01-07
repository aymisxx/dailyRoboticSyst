import numpy as np


class MassSpringDamper:
    def __init__(self, m: float, c: float, k: float):
        if m <= 0:
            raise ValueError("Mass m must be > 0.")
        self.m = float(m)
        self.c = float(c)
        self.k = float(k)

    def dynamics(self, t: float, state, u: float = 0.0) -> np.ndarray:
        x, x_dot = state
        x_ddot = (u - self.c * x_dot - self.k * x) / self.m
        return np.array([x_dot, x_ddot], dtype=float)