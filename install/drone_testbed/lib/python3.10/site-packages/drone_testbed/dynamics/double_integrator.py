"""Double integrator dynamics model for 2D drone simulation.

State:   [x, y, vx, vy]
Control: [ax, ay]

Matches the reference repo's dynamics:
  x_{t+1} = x_t + vx_t * dt + 0.5 * ax * dt^2
  vx_{t+1} = vx_t + ax * dt
"""

import numpy as np


def step(state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
    """Advance one timestep of double integrator dynamics.

    Args:
        state: [x, y, vx, vy]
        control: [ax, ay] (clipped to acceleration limits internally)
        dt: timestep in seconds

    Returns:
        New state [x, y, vx, vy]
    """
    x, y, vx, vy = state
    ax, ay = control

    return np.array([
        x + vx * dt + 0.5 * ax * dt ** 2,
        y + vy * dt + 0.5 * ay * dt ** 2,
        vx + ax * dt,
        vy + ay * dt,
    ])


# State-space matrices (for algorithms that need them)
def get_matrices(dt: float):
    """Return (A, B) matrices for the double integrator.

    x_{t+1} = A @ x_t + B @ u_t
    """
    A = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    B = np.array([
        [0.5 * dt ** 2, 0],
        [0, 0.5 * dt ** 2],
        [dt, 0],
        [0, dt],
    ])
    return A, B
