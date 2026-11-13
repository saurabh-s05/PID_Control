import numpy as np


class LinearPlant:
    """x' = A x + B u,  y = C x  — integrated with RK4."""

    def __init__(self, A, B, C):
        self.A = np.array(A, dtype=float)
        self.B = np.array(B, dtype=float)
        self.C = np.array(C, dtype=float)
        self.reset()

    def reset(self):
        self.x = np.zeros(self.A.shape[0])

    def output(self):
        return float(self.C @ self.x)

    def _f(self, x, u):
        return self.A @ x + self.B * u

    def step(self, u, dt):
        x = self.x
        k1 = self._f(x, u)
        k2 = self._f(x + 0.5 * dt * k1, u)
        k3 = self._f(x + 0.5 * dt * k2, u)
        k4 = self._f(x + dt * k3, u)
        self.x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return self.output()


class MassSpringDamper(LinearPlant):
    """m x'' + c x' + k x = F.  States: [position, velocity]."""

    def __init__(self, m=1.0, c=10.0, k=20.0):
        self.m, self.c, self.k = m, c, k
        A = [[0, 1],
             [-k / m, -c / m]]
        B = [0, 1 / m]
        C = [1, 0]
        super().__init__(A, B, C)


class ThirdOrderLag(LinearPlant):
    """G(s) = 1 / (s+1)^3 = 1 / (s^3 + 3s^2 + 3s + 1)."""

    def __init__(self):
        A = [[0, 1, 0],
             [0, 0, 1],
             [-1, -3, -3]]
        B = [0, 0, 1]
        C = [1, 0, 0]
        super().__init__(A, B, C)
