"""Leader-follower formation control algorithm.

The leader drone follows a predefined circular path.
Follower drones maintain fixed offsets from the leader using PD control.

Config params:
  leader_id: which drone is the leader (default: first drone)
  leader_speed: angular speed of leader on its path (rad/s)
  leader_radius: radius of leader's circular path (m)
  formation_offsets: dict of drone_id -> [dx, dy] offset from leader
  gain_kp: proportional gain for position error
  gain_kd: derivative gain for velocity damping
"""

import math
from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


@register_algorithm
class LeaderFollower(BaseAlgorithm):

    def __init__(self):
        self._leader_id = None
        self._follower_ids = []
        self._offsets: Dict[str, np.ndarray] = {}
        self._kp = 1.5
        self._kd = 0.8
        self._leader_speed = 0.3
        self._leader_radius = 0.8
        self._time = 0.0

    def name(self) -> str:
        return "LeaderFollower"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._kp = params.get('gain_kp', 1.5)
        self._kd = params.get('gain_kd', 0.8)
        self._leader_speed = params.get('leader_speed', 0.3)
        self._leader_radius = params.get('leader_radius', 0.8)
        self._leader_id = params.get('leader_id', drone_ids[0])
        self._time = 0.0

        # Build formation offsets
        offsets_cfg = params.get('formation_offsets', {})
        self._follower_ids = [d for d in drone_ids if d != self._leader_id]

        # If offsets not provided, arrange followers in a regular polygon behind leader
        for i, drone_id in enumerate(self._follower_ids):
            if drone_id in offsets_cfg:
                self._offsets[drone_id] = np.array(offsets_cfg[drone_id], dtype=float)
            else:
                angle = 2.0 * math.pi * (i + 1) / (len(self._follower_ids) + 1)
                self._offsets[drone_id] = np.array([
                    0.6 * math.cos(angle),
                    0.6 * math.sin(angle),
                ])

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        self._time += dt
        controls = {}

        # Leader: follow a circular path
        t = self._time
        r = self._leader_radius
        w = self._leader_speed

        # Desired leader position and velocity on circle
        target_pos = np.array([r * math.cos(w * t), r * math.sin(w * t)])
        target_vel = np.array([-r * w * math.sin(w * t), r * w * math.cos(w * t)])

        if self._leader_id in states:
            leader_state = states[self._leader_id]
            pos_err = target_pos - leader_state.position
            vel_err = target_vel - leader_state.velocity
            accel = self._kp * pos_err + self._kd * vel_err
            controls[self._leader_id] = ControlOutput(
                drone_id=self._leader_id,
                acceleration=np.clip(accel, -0.5, 0.5),
            )

        # Followers: track leader + offset with PD control
        leader_pos = states[self._leader_id].position if self._leader_id in states else np.zeros(2)
        leader_vel = states[self._leader_id].velocity if self._leader_id in states else np.zeros(2)

        for drone_id in self._follower_ids:
            if drone_id not in states:
                continue

            desired_pos = leader_pos + self._offsets[drone_id]
            desired_vel = leader_vel

            pos_err = desired_pos - states[drone_id].position
            vel_err = desired_vel - states[drone_id].velocity
            accel = self._kp * pos_err + self._kd * vel_err
            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -0.5, 0.5),
            )

        return controls

    def reset(self) -> None:
        self._time = 0.0
