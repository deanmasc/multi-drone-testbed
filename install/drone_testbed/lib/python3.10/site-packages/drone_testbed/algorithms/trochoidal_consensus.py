"""Trochoidal patterns from a distributed consensus law.

Implements the control law of Monsingh, Sinha & Chung, "Trochoidal patterns
generation using generalized consensus strategy for double-integrator dynamic
agents", European Journal of Control 76 (2024) 100928, eq. (7), in 2-D.

    u_i = -alpha*p_i - beta*v_i - kappa*R(theta) * sum_{j in N(i)} a_ij (p_i - p_j)

The difference from `trochoidal.py` is the whole point. That one *hard-codes*
the path: every drone is told its exact position at every instant, and the
pattern exists because the paths were designed to fit together. Here no drone
is told anything. Each one knows only its own position and velocity, and the
relative positions of its neighbours. The trochoid is emergent -- it appears in
the closed-loop dynamics, not in the reference.

Why it works, briefly. Stack the agents and the closed loop is

    [p_dot; v_dot] = A [p; v],   A = [ 0                            I      ]
                                     [ -alpha*I - kappa*(L (x) R)  -beta*I ]

with L the graph Laplacian. Choose the gains so that A has exactly TWO distinct
pairs of eigenvalues on the imaginary axis and every other pole strictly in the
left half plane. The stable modes die out; the two undamped modes survive as two
superimposed circular motions -- which is precisely the parametric form of a
trochoid. The two frequencies are the two circles; the radii come from the
initial conditions.

Consequences worth knowing before flying it:

  * It needs at least 3 agents. With one, L is zero, the coupling term
    vanishes, and all that is left is a damped oscillator decaying to the
    origin. No pattern is possible.
  * The gains are NOT free. They must be computed from the graph's eigenvalues
    to place those poles. Use tools/design_consensus_gains.py -- picking them by
    hand gives decay to a point, a plain circle, or a divergent spiral.
  * You cannot ask for a radius. Size comes from the initial conditions, so
    check the predicted envelope against the geofence before arming.
  * There is no reference trajectory, so "tracking error" is undefined. Validate
    it by checking the observed frequencies and radii against the eigenvalue
    prediction instead.

Config params:
  alpha:     absolute position gain (centres the pattern on the origin)
  beta:      absolute velocity gain (damping)
  kappa:     neighbour coupling gain
  theta:     Cartesian coupling angle, radians. This is the paper's
             contribution: without the rotation the motion is only
             trochoid-LIKE, and agents starting from identical states collapse
             onto a straight line.
  adjacency: which agents each drone observes. Two accepted forms:
                 drone1: ["drone3"]              unweighted, a_ij = 1
                 drone1: {drone3: 5.0}           weighted, a_ij = 5.0
             Weights are the a_ij of the paper's control law and are what let
             agents end up on DIFFERENT annuli. An unweighted directed cycle --
             the default -- has a circulant Laplacian, whose eigenvectors all
             have equal modulus, so every agent necessarily gets the SAME
             radius and only its phase differs (the paper's Remark 4.2: the
             cyclic case degenerates to epicycloids). Break that symmetry to
             get the varied pattern of the paper's Fig. 4.
             Default is an unweighted directed cycle over the config order.
"""

import math
from typing import Dict, List

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


def _weighted(neighbours) -> Dict[str, float]:
    """Normalise either adjacency form to {neighbour_id: weight}.

    A bare list is the unweighted case, a_ij = 1 for every listed neighbour.
    """
    if isinstance(neighbours, dict):
        return {str(k): float(v) for k, v in neighbours.items()}
    return {str(k): 1.0 for k in neighbours}


@register_algorithm
class TrochoidalConsensus(BaseAlgorithm):

    def __init__(self):
        self._alpha = 1.119
        self._beta = 1.0
        self._kappa = 0.591
        self._theta = 2.17
        self._adjacency: Dict[str, Dict[str, float]] = {}
        self._max_accel = 0.5
        self._warned = False

    def name(self) -> str:
        return "TrochoidalConsensus"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._alpha = params.get('alpha', 1.119)
        self._beta = params.get('beta', 1.0)
        self._kappa = params.get('kappa', 0.591)
        self._theta = params.get('theta', 2.17)
        self._max_accel = params.get('max_accel', 0.5)

        adjacency = params.get('adjacency')
        if adjacency:
            self._adjacency = {
                d: _weighted(adjacency.get(d, [])) for d in drone_ids
            }
        else:
            # Directed cycle: each agent observes the next one round. Note this
            # default cannot produce per-agent radius variation -- see the
            # module docstring.
            n = len(drone_ids)
            self._adjacency = {
                d: {drone_ids[(i + 1) % n]: 1.0}
                for i, d in enumerate(drone_ids)
            }
        self._warned = False

    def _rotation(self) -> np.ndarray:
        c, s = math.cos(self._theta), math.sin(self._theta)
        return np.array([[c, -s], [s, c]])

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        rot = self._rotation()
        controls = {}

        if len(states) < 3 and not self._warned:
            self._warned = True
            print('[TrochoidalConsensus] fewer than 3 agents -- the coupling '
                  'term cannot produce a pattern; expect decay to the origin.')

        for drone_id, state in states.items():
            neighbours = self._adjacency.get(drone_id, {})

            # Relative position to neighbours: the ONLY non-local information
            # used. Everything else below is the agent's own state.
            relative = np.zeros(2)
            for neighbour_id, weight in neighbours.items():
                other = states.get(neighbour_id)
                if other is None:
                    continue
                relative += weight * (state.position - other.position)

            accel = (
                -self._alpha * state.position
                - self._beta * state.velocity
                - self._kappa * (rot @ relative)
            )

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
                # No position/velocity setpoint on purpose: there is no
                # reference trajectory to publish. crazyflie_node integrates the
                # acceleration, which is the correct reading of a double
                # integrator's control input.
            )

        return controls

    def reset(self) -> None:
        self._warned = False
