"""Trochoidal distributed motion algorithm.

Each drone independently follows a trochoidal path -- the superposition of
two circular motions:
  1. A slow large orbit (all drones evenly phase-spaced around a common centre)
  2. A fast small epicycle (each drone spins its own loop on top of the orbit)

This is truly distributed: each drone computes its own target using only its
assigned phase offset. No inter-drone communication is required.

The resulting collective pattern looks like interlocking petal/flower curves.

Config params:
  orbit_radius:    radius of the main orbit (m), default 0.8
  epicycle_radius: radius of the per-drone epicycle (m), default 0.3
  orbit_speed:     angular velocity of main orbit (rad/s), default 0.3
  epicycle_speed:  angular velocity of epicycle (rad/s), default 1.2
  gain_kp:         PD proportional gain, default 1.5
  gain_kd:         PD derivative gain, default 0.8
  anchor_on_start: shift the whole pattern so it begins where the drones
                   actually are, default True. False pins it to the world
                   origin, which requires placing the drone at
                   (orbit_radius + epicycle_radius, 0) by hand.
"""

import math
from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


@register_algorithm
class Trochoidal(BaseAlgorithm):

    def __init__(self):
        self._phases: Dict[str, float] = {}
        self._orbit_radius = 0.8
        self._epicycle_radius = 0.3
        self._orbit_speed = 0.3
        self._epicycle_speed = 1.2
        self._kp = 1.5
        self._kd = 0.8
        self._time = 0.0
        self._anchor = True
        # Translation applied to the whole pattern, fixed on the first tick.
        # Kept a real vector at all times so _target is safe to call before
        # then -- _set_offset itself needs it while computing the offset.
        self._offset = np.zeros(2)
        self._anchored = False

    def name(self) -> str:
        return "Trochoidal"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._orbit_radius = params.get('orbit_radius', 0.8)
        self._epicycle_radius = params.get('epicycle_radius', 0.3)
        self._orbit_speed = params.get('orbit_speed', 0.3)
        self._epicycle_speed = params.get('epicycle_speed', 1.2)
        self._kp = params.get('gain_kp', 1.5)
        self._kd = params.get('gain_kd', 0.8)
        self._anchor = params.get('anchor_on_start', True)
        self._time = 0.0
        self._offset = np.zeros(2)
        self._anchored = False

        # Assign each drone an evenly spaced phase offset around the orbit
        n = len(drone_ids)
        for i, drone_id in enumerate(drone_ids):
            self._phases[drone_id] = 2.0 * math.pi * i / n

    def _target(self, phase: float, t: float):
        """Compute target position and velocity for a given phase at time t."""
        R = self._orbit_radius
        r = self._epicycle_radius
        W = self._orbit_speed       # orbit angular velocity
        w = self._epicycle_speed    # epicycle angular velocity

        # Trochoidal position: large orbit + small epicycle, translated by the
        # anchor offset so the pattern sits where the drones started.
        pos = self._offset + np.array([
            R * math.cos(W * t + phase) + r * math.cos(w * t + phase),
            R * math.sin(W * t + phase) + r * math.sin(w * t + phase),
        ])

        # Analytical velocity (derivative of pos w.r.t. t)
        vel = np.array([
            -R * W * math.sin(W * t + phase) - r * w * math.sin(w * t + phase),
             R * W * math.cos(W * t + phase) + r * w * math.cos(w * t + phase),
        ])

        return pos, vel

    def _set_offset(self, states: Dict[str, DroneState]) -> None:
        """Fix where the pattern sits, once, from the drones' real positions.

        Unlike Square, this path is a closed shape around a centre rather than
        one that starts under the drone, so at t=0 its natural start is
        (orbit_radius + epicycle_radius, 0) -- and a drone placed anywhere else
        gets a saturated command on the very first tick and lunges across the
        volume to catch up.

        One shared offset is applied to every drone rather than anchoring each
        on itself, because the formation is the point: the drones are evenly
        phase-spaced around a *common* orbit, and anchoring individually would
        scatter them into separate overlapping patterns. For evenly spaced
        phases the untranslated positions sum to zero, so matching centroids
        reduces to centring the pattern on the drones -- and for a single drone
        it puts the path start exactly under it, giving zero initial error.
        """
        self._offset = np.zeros(2)
        self._anchored = True
        if not self._anchor:
            return
        actual = np.mean([s.position for s in states.values()], axis=0)
        nominal = np.mean(
            [self._target(self._phases[d], 0.0)[0]
             for d in states if d in self._phases],
            axis=0,
        )
        self._offset = actual - nominal

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        # Must happen before the clock advances, so the anchor is computed
        # against t=0 -- the same instant the drones are sitting still at.
        if not self._anchored:
            self._set_offset(states)

        self._time += dt
        controls = {}

        for drone_id, state in states.items():
            if drone_id not in self._phases:
                continue

            phase = self._phases[drone_id]
            target_pos, target_vel = self._target(phase, self._time)

            pos_err = target_pos - state.position
            vel_err = target_vel - state.velocity

            accel = self._kp * pos_err + self._kd * vel_err
            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -0.5, 0.5),
                # This is a trajectory-following algorithm: it knows the exact
                # intended state, so send it rather than making crazyflie_node
                # rebuild the path by integrating the acceleration above. The
                # simulator ignores these.
                position=target_pos,
                velocity=target_vel,
            )

        return controls

    def reset(self) -> None:
        self._time = 0.0
        self._offset = np.zeros(2)
        self._anchored = False
