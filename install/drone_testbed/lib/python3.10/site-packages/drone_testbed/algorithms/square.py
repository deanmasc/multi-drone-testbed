"""Square path -- single-drone motion test.

Traces a square in the x/y plane at constant speed, holding altitude. Intended
as the first algorithm to run after PrintState: it is the simplest thing that
actually commands motion, and a square is easy to judge by eye -- straight
sides, square corners, returns to where it started.

The path begins at the drone's position when the algorithm starts, so there is
no position error on the first tick and therefore no lunge. It then runs +x, +y,
-x, -y with each corner rounded into an arc.

The start point sits one corner_radius along the first side, so the box swept is
[x0 - r, x0 - r + side] x [y0, y0 + side] -- shifted by r in x relative to the
start. Check that box is inside your flight volume before running.

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
  corner_radius: radius of the arc replacing each corner (m), default 0.15.
                0 restores hard corners, which demand infinite acceleration and
                make the drone jerk. Capped at side/2.
  log_file:     path to write to; empty = timestamped file in the working
                directory, default ''
  log_interval: seconds between samples, default 0.1. Cannot resolve finer
                than the control period (1 / control_rate).
"""

import math
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
        self._corner_radius = 0.15
        self._segments = None
        self._perimeter = 0.0
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
        self._corner_radius = params.get('corner_radius', 0.15)
        self._log_interval = params.get('log_interval', 0.1)
        self.reset()
        self._open_log(params.get('log_file', ''))

    def _build_path(self, origin: np.ndarray):
        """Segments of the rounded square, walked at constant speed.

        A true square corner asks the drone to reverse its velocity in one tick,
        which is an infinite acceleration demand. The firmware answers by
        slamming the attitude over -- the jerk you can see in flight, and the
        moment the drone is least stable. Replacing each corner with a quarter
        circle keeps the speed constant and turns the velocity smoothly, so the
        commanded path is one the drone can actually fly.

        corner_radius 0 gives the old sharp-cornered square back.

        The path starts at `origin`, which sits on the first side one radius
        past the nominal corner -- so the drone begins with zero error and the
        lap closes exactly where it started.
        """
        radius = min(self._corner_radius, self._side / 2.0)
        # Nominal square corner, placed so the path begins at origin.
        base = origin - np.array([radius, 0.0])
        corners = [base + c * self._side for c in _CORNERS]
        dirs = [
            (corners[(i + 1) % 4] - corners[i]) / self._side for i in range(4)
        ]

        straight_len = self._side - 2.0 * radius
        arc_len = 0.5 * math.pi * radius

        segments = []
        for i in range(4):
            nxt = corners[(i + 1) % 4]
            segments.append({
                'kind': 'line',
                'start': corners[i] + radius * dirs[i],
                'dir': dirs[i],
                'length': straight_len,
            })
            if radius > 0.0:
                # Centre sits one radius inside both the incoming and outgoing
                # edges, so the arc is tangent to each -- that tangency is what
                # makes velocity continuous through the corner.
                centre = nxt - radius * dirs[i] + radius * dirs[(i + 1) % 4]
                start_vec = -dirs[(i + 1) % 4]
                segments.append({
                    'kind': 'arc',
                    'centre': centre,
                    'radius': radius,
                    'start_angle': math.atan2(start_vec[1], start_vec[0]),
                    'length': arc_len,
                })

        self._segments = segments
        self._perimeter = sum(s['length'] for s in segments)

    def _target(self, origin: np.ndarray, t: float):
        """Target position and velocity on the path at time t.

        Before start_delay elapses the target is the origin, held still, so the
        drone settles after takeoff instead of accelerating straight out of it.
        """
        if t < self._start_delay:
            return origin.copy(), np.zeros(2)

        if self._segments is None:
            self._build_path(origin)

        # Constant speed, so distance along the path is just speed * time.
        distance = (self._speed * (t - self._start_delay)) % self._perimeter

        for seg in self._segments:
            if distance > seg['length']:
                distance -= seg['length']
                continue
            if seg['kind'] == 'line':
                pos = seg['start'] + distance * seg['dir']
                vel = self._speed * seg['dir']
            else:
                # Quarter turn, counter-clockwise, at constant speed.
                angle = seg['start_angle'] + distance / seg['radius']
                pos = seg['centre'] + seg['radius'] * np.array(
                    [math.cos(angle), math.sin(angle)]
                )
                vel = self._speed * np.array([-math.sin(angle), math.cos(angle)])
            return pos, vel

        # Only reachable through float error at the very end of a lap.
        seg = self._segments[-1]
        return self._target(origin, self._start_delay)

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
        self._segments = None
