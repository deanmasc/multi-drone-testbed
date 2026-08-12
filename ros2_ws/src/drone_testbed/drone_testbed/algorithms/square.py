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

Logs actual and target position to a text file so the flown path can be
compared against the commanded one after the fact -- the gap between the two
is the thing to judge, not the raw track.

Config params:
  side:         square side length (m), default 0.5
  speed:        traverse speed along each side (m/s), default 0.15
  start_delay:  seconds to hold position before moving, default 3.0
  gain_kp:      PD proportional gain, default 1.0
  gain_kd:      PD derivative gain, default 0.6
  max_accel:    acceleration clamp (m/s^2), default 0.3
  log_file:     path to write to; empty = timestamped file in the working
                directory, default ''
  log_interval: seconds between samples, default 0.1. Cannot resolve finer
                than the control period (1 / control_rate).
"""

import os
from datetime import datetime
from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


# Unit square corners, traversed in order and wrapping back to the first.
# Scaled by `side` at use. Keep these 0/1 -- changing them desynchronises the
# leg length from side_duration, which silently scales the commanded speed.
# To change the square's size, set `side` in the config.
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
        self._log = None
        self._log_interval = 0.1
        self._next_log = 0.0

    def name(self) -> str:
        return "Square"

    def _open_log(self, path: str) -> None:
        """Open the flight log, or carry on without one if it cannot be made."""
        self._close_log()
        if not path:
            path = f'square_log_{datetime.now():%Y%m%d_%H%M%S}.txt'
        path = os.path.abspath(os.path.expanduser(path))
        try:
            # Line buffered: a run ended with Ctrl+C still leaves a complete
            # file rather than losing whatever was sitting in the buffer.
            self._log = open(path, 'w', buffering=1)
        except OSError as exc:
            print(f'[Square] could not open log file {path}: {exc}')
            return
        self._log.write(
            '# Square flight log. Positions in metres, time in seconds\n'
            '# since the algorithm started (motion begins at start_delay).\n'
            '# t drone_id x y target_x target_y error\n'
        )
        print(f'[Square] logging to {path}')

    def _close_log(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._side = params.get('side', 0.5)
        self._speed = params.get('speed', 0.15)
        self._start_delay = params.get('start_delay', 3.0)
        self._kp = params.get('gain_kp', 1.0)
        self._kd = params.get('gain_kd', 0.6)
        self._max_accel = params.get('max_accel', 0.3)
        self._log_interval = params.get('log_interval', 0.1)
        self.reset()
        self._open_log(params.get('log_file', ''))

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

        # Sampling is driven by the control tick, so the interval is rounded up
        # to a whole number of ticks -- ask for 0.1s at 10Hz and you get every
        # tick; ask for 0.5s and you get every fifth.
        # Advance the deadline along a fixed grid rather than from the current
        # time: self._time accumulates float error, and rescheduling off it
        # lets every sample slip a whole tick late. Epsilon absorbs the case
        # where the accumulated time lands a hair under an exact grid point.
        due = self._log is not None and self._time + 1e-9 >= self._next_log
        if due:
            self._next_log += self._log_interval
            if self._next_log < self._time:
                # Fell behind (control loop stalled) -- resync to the grid.
                self._next_log = self._time + self._log_interval

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
                # We know the exact intended state, so publish it rather than
                # making crazyflie_node rebuild it by integrating the
                # acceleration above. The simulator ignores these.
                position=target_pos,
                velocity=target_vel,
            )

            if due:
                error = float(np.linalg.norm(target_pos - state.position))
                self._log.write(
                    f'{self._time:7.2f} {drone_id} '
                    f'{state.position[0]:+8.4f} {state.position[1]:+8.4f} '
                    f'{target_pos[0]:+8.4f} {target_pos[1]:+8.4f} '
                    f'{error:7.4f}\n'
                )

        return controls

    def reset(self) -> None:
        self._time = 0.0
        self._next_log = 0.0
        self._origins.clear()
