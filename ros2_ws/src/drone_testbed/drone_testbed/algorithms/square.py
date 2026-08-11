"""Square path -- single-drone motion test.

Traces a square in the x/y plane at constant speed, holding altitude. Intended
as the first algorithm to run after PrintState: it is the simplest thing that
actually commands motion, and a square is easy to judge by eye -- straight
sides, square corners, returns to where it started.

The square's first corner is the drone's position when the algorithm starts, so
there is no position error on the first tick and therefore no lunge. The path
then runs +x, +y, -x, -y, i.e. the square occupies
[x0, x0 + side] x [y0, y0 + side]. Check that box is inside your flight volume
before running.

Config params:
  side:        square side length (m), default 0.5
  speed:       traverse speed along each side (m/s), default 0.15
  start_delay: seconds to hold position before moving, default 3.0
  gain_kp:     PD proportional gain, default 1.0
  gain_kd:     PD derivative gain, default 0.6
  max_accel:   acceleration clamp (m/s^2), default 0.3
"""

from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


# Unit square corners, traversed in order and wrapping back to the first.
_CORNERS = np.array([
    [0.0, 0.0],
    [1.0, 0.0],
    [1.0, 1.0],
    [0.0, 1.0],
])


@register_algorithm
class Square(BaseAlgorithm):

    def __init__(self):
        self._side = 0.5
        self._speed = 0.15
        self._start_delay = 3.0
        self._kp = 1.0
        self._kd = 0.6
        self._max_accel = 0.3
        self._time = 0.0
        self._origins: Dict[str, np.ndarray] = {}

    def name(self) -> str:
        return "Square"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._side = params.get('side', 0.5)
        self._speed = params.get('speed', 0.15)
        self._start_delay = params.get('start_delay', 3.0)
        self._kp = params.get('gain_kp', 1.0)
        self._kd = params.get('gain_kd', 0.6)
        self._max_accel = params.get('max_accel', 0.3)
        self.reset()

    def _target(self, origin: np.ndarray, t: float):
        """Target position and velocity on the square at time t.

        Before start_delay elapses the target is the origin, held still, so the
        drone settles after takeoff instead of accelerating straight out of it.
        """
        if t < self._start_delay:
            return origin.copy(), np.zeros(2)

        side_duration = self._side / self._speed
        elapsed = t - self._start_delay

        # Which side we are on, and how far along it (0..1).
        index = int(elapsed // side_duration) % 4
        frac = (elapsed % side_duration) / side_duration

        here = _CORNERS[index] * self._side
        nxt = _CORNERS[(index + 1) % 4] * self._side
        leg = nxt - here

        pos = origin + here + frac * leg
        vel = leg / side_duration
        return pos, vel

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        self._time += dt
        controls = {}

        for drone_id, state in states.items():
            # Anchor the square wherever the drone actually was when we started,
            # so the first tick has zero position error.
            if drone_id not in self._origins:
                self._origins[drone_id] = state.position.copy()

            target_pos, target_vel = self._target(
                self._origins[drone_id], self._time,
            )

            accel = (
                self._kp * (target_pos - state.position)
                + self._kd * (target_vel - state.velocity)
            )

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
            )

        return controls

    def reset(self) -> None:
        self._time = 0.0
        self._origins.clear()
