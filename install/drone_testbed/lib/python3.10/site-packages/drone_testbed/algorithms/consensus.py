"""Consensus-based formation control algorithm.

Each drone computes its acceleration using only neighbor information:
  a_i = kp * sum_j(w_ij * ((p_j - p_i) - (d_j - d_i)))
       + kd * sum_j(w_ij * (v_j - v_i))

where d_i are desired formation offsets and w_ij are adjacency weights.
This is a truly distributed algorithm -- each drone only reads neighbors.

Config params:
  adjacency: dict of drone_id -> list of neighbor drone_ids
             (default: fully connected graph)
  target_offsets: dict of drone_id -> [dx, dy] desired position in formation
  consensus_gain_kp: position consensus gain
  consensus_gain_kd: velocity consensus gain
"""

import math
from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


@register_algorithm
class ConsensusFormation(BaseAlgorithm):

    def __init__(self):
        self._adjacency: Dict[str, List[str]] = {}
        self._offsets: Dict[str, np.ndarray] = {}
        self._kp = 1.0
        self._kd = 0.8

    def name(self) -> str:
        return "ConsensusFormation"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._kp = params.get('consensus_gain_kp', 1.0)
        self._kd = params.get('consensus_gain_kd', 0.8)

        # Adjacency graph: default to fully connected
        adj_cfg = params.get('adjacency', {})
        if adj_cfg:
            self._adjacency = {k: list(v) for k, v in adj_cfg.items()}
        else:
            self._adjacency = {
                d: [other for other in drone_ids if other != d]
                for d in drone_ids
            }

        # Target formation offsets: default to regular polygon
        offsets_cfg = params.get('target_offsets', {})
        n = len(drone_ids)
        for i, drone_id in enumerate(drone_ids):
            if drone_id in offsets_cfg:
                self._offsets[drone_id] = np.array(offsets_cfg[drone_id], dtype=float)
            else:
                angle = 2.0 * math.pi * i / n
                self._offsets[drone_id] = np.array([
                    0.8 * math.cos(angle),
                    0.8 * math.sin(angle),
                ])

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        controls = {}

        for drone_id, neighbors in self._adjacency.items():
            if drone_id not in states:
                continue

            my_state = states[drone_id]
            my_offset = self._offsets.get(drone_id, np.zeros(2))

            pos_sum = np.zeros(2)
            vel_sum = np.zeros(2)
            n_neighbors = 0

            for neighbor_id in neighbors:
                if neighbor_id not in states:
                    continue
                neighbor_state = states[neighbor_id]
                neighbor_offset = self._offsets.get(neighbor_id, np.zeros(2))

                # Position consensus: drive relative positions toward desired offsets
                actual_diff = neighbor_state.position - my_state.position
                desired_diff = neighbor_offset - my_offset
                pos_sum += actual_diff - desired_diff

                # Velocity consensus: match neighbor velocities
                vel_sum += neighbor_state.velocity - my_state.velocity
                n_neighbors += 1

            if n_neighbors > 0:
                accel = self._kp * pos_sum / n_neighbors + self._kd * vel_sum / n_neighbors
            else:
                accel = np.zeros(2)

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -0.5, 0.5),
            )

        return controls

    def reset(self) -> None:
        pass
