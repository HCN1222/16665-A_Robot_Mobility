"""Models transcribed from the assignment and Lectures 4 and 6."""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class Vehicle:
    speed: float = 30.0
    mass: float = 1573.0
    inertia: float = 2873.0
    front: float = 1.1
    rear: float = 1.58
    cf: float = 80000.0
    cr: float = 80000.0

    def error_matrices(self):
        v, m, j = self.speed, self.mass, self.inertia
        a, b, f, r = self.front, self.rear, self.cf, self.cr
        A = np.array([
            [0, 1, 0, 0],
            [0, -2*(f+r)/(m*v), 2*(f+r)/m, -2*(f*a-r*b)/(m*v)],
            [0, 0, 0, 1],
            [0, -2*(f*a-r*b)/(j*v), 2*(f*a-r*b)/j,
             -2*(f*a*a+r*b*b)/(j*v)]])
        B = np.array([0, 2*f/m, 0, 2*f*a/j])
        D = np.array([0, -2*(f*a-r*b)/(m*v)-v, 0,
                      -2*(f*a*a+r*b*b)/(j*v)])
        return A, B, D


def lqr(vehicle, diagonal):
    A, B, D = vehicle.error_matrices()
    Q = np.diag(diagonal)
    P = solve_continuous_are(A, B[:, None], Q, np.ones((1, 1)))
    K = B@P
    residual = A.T@P+P@A-np.outer(P@B, B@P)+Q
    assert np.max(abs(residual)) < 1e-8
    controllability = np.column_stack([np.linalg.matrix_power(A, k)@B
                                      for k in range(4)])
    assert np.linalg.matrix_rank(controllability) == 4
    return A-np.outer(B, K), D, K


def pepy(state, velocity, steering, wheelbase=3.):
    yaw = state[2]
    return np.array([velocity*np.cos(yaw), velocity*np.sin(yaw),
                     velocity*np.tan(steering)/wheelbase])
