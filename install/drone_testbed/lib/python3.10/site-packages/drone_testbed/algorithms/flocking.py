"""Flocking with a virtual leader.

Implements Olfati-Saber, "Flocking for Multi-Agent Dynamic Systems: Algorithms
and Theory", IEEE TAC 51(3) 2006, Algorithm 2 -- the free-flocking law plus
navigational feedback toward a gamma-agent.

    u_i = u_i^alpha + u_i^gamma

    u_i^alpha = c1a * sum_{j in N_i} phi_alpha(||q_j - q_i||_sigma) * n_ij
              + c2a * sum_{j in N_i} a_ij(q) * (p_j - p_i)

    u_i^gamma = -c1g * (q_i - q_gamma) - c2g * (p_i - p_gamma)

The name is literal: this is Reynolds' 1987 boids rules written as a control
law. The first alpha term is separation-and-cohesion (push apart when closer
than `spacing`, pull together when further), the second is velocity matching,
and the gamma term is the shared sense of where the group is heading.

Two things distinguish it from every other algorithm in this directory:

  * The communication graph is NOT configured, it is a function of the state.
    Agent i's neighbours are whoever is currently within `sense_range` of it, so
    the graph forms, breaks and reforms as the drones move. `adjacency` in the
    YAML does nothing here -- there is nothing to put in it.
  * The interaction is a nonlinear potential, not a linear spring. The pairwise
    force saturates (bounded by the `a`/`b` pair) instead of growing without
    limit, and it goes to zero smoothly at the edge of sensing range via the
    bump function, so a neighbour entering or leaving range does not produce a
    step change in commanded acceleration.

Honest limitation, worth stating in a report: u_i^gamma gives every agent the
same q_gamma and p_gamma, which is global broadcast information. Only the alpha
terms are strictly local. With `gamma_mode: static` the gamma term degenerates
to "everybody knows where the origin is" -- the same absolute-position feedback
TrochoidalConsensus uses -- and the flock forms a lattice and parks in it.

What to expect with 4 drones: they pull together, overshoot, settle into a
rhombus roughly `spacing` metres on a side (4 agents is too few for the regular
triangular lattice the theory predicts asymptotically), and then translate as a
rigid shape while tracking the gamma-agent. Relative motion stops once the
lattice forms, so all four trace the SAME curve at fixed offsets -- unlike
TrochoidalConsensus, where each agent gets a genuinely different path.

Config params:
  spacing:          desired inter-agent distance d, metres
  sense_range:      interaction radius r, metres. Must exceed `spacing`;
                    r/d around 1.2-1.5 is the usual choice. Note that with a
                    small number of agents the settled separation comes out
                    BELOW `spacing` -- everyone is inside everyone's range, so
                    each agent is pulled by several neighbours at once and the
                    shape compresses. At 4 agents and r/d = 1.2 the measured
                    factor is 0.84. Size the lattice by measuring, not by
                    reading `spacing`.
  epsilon:          sigma-norm parameter. Small values make the sigma-norm
                    closer to the true Euclidean norm but stiffen the gradient.
  bump_h:           bump function knee in (0,1). Below h the weight is 1, from
                    h to 1 it eases to 0 on a raised cosine.
  action_a,
  action_b:         shape the pairwise action function, 0 < a <= b. a == b
                    gives an odd (symmetric push/pull) response.
  gain_c1_alpha:    gradient (spacing) gain
  gain_c2_alpha:    velocity-matching gain
  gain_c1_gamma:    navigational position gain
  gain_c2_gamma:    navigational velocity gain
  gamma_mode:       'circle' | 'line' | 'static' -- the virtual leader's path
  gamma_center:     [x, y] centre of the circle, or the static hold point
  gamma_radius:     circle radius, metres (circle mode)
  gamma_speed:      leader speed along the circle, m/s (circle mode)
  gamma_heading:    direction of travel in radians (line mode)
  max_accel:        per-axis acceleration clamp, m/s^2
"""

import math
from typing import Dict, List, Tuple

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


def _sigma_norm(z: np.ndarray, epsilon: float) -> float:
    """The paper's sigma-norm: smooth everywhere including at z = 0.

    The Euclidean norm is not differentiable at the origin, which would make the
    pairwise force discontinuous exactly when two agents collide. This is the
    standard fix.
    """
    return (math.sqrt(1.0 + epsilon * float(z @ z)) - 1.0) / epsilon


def _sigma_grad(z: np.ndarray, epsilon: float) -> np.ndarray:
    """Gradient of the sigma-norm -- a softened unit vector along z."""
    return z / math.sqrt(1.0 + epsilon * float(z @ z))


def _bump(z: float, h: float) -> float:
    """Raised-cosine bump: 1 below h, easing to 0 at 1, 0 beyond.

    This is what makes the neighbour set a smooth function of position rather
    than an on/off switch at the sensing radius.
    """
    if z < 0.0:
        return 0.0
    if z < h:
        return 1.0
    if z <= 1.0:
        return 0.5 * (1.0 + math.cos(math.pi * (z - h) / (1.0 - h)))
    return 0.0


def _sigma_1(z: float) -> float:
    """z / sqrt(1 + z^2) -- a smooth saturating sign function."""
    return z / math.sqrt(1.0 + z * z)


@register_algorithm
class Flocking(BaseAlgorithm):

    def __init__(self):
        self._d = 0.6
        self._r = 0.9
        self._epsilon = 0.1
        self._h = 0.2
        self._a = 5.0
        self._b = 5.0
        self._c1a = 1.0
        self._c2a = 2.0
        self._c1g = 0.5
        self._c2g = 1.4
        self._gamma_mode = 'circle'
        self._gamma_center = np.zeros(2)
        self._gamma_radius = 1.0
        self._gamma_speed = 0.15
        self._gamma_heading = 0.0
        self._max_accel = 0.5

        # Derived once in configure -- these are the sigma-norm images of the
        # spacing and sensing radius, and all the distance comparisons below
        # happen in sigma-norm space, not metres.
        self._d_alpha = 0.0
        self._r_alpha = 0.0
        self._c = 0.0

        self._time = 0.0

    def name(self) -> str:
        return "Flocking"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._d = params.get('spacing', 0.6)
        self._r = params.get('sense_range', 0.9)
        self._epsilon = params.get('epsilon', 0.1)
        self._h = params.get('bump_h', 0.2)
        self._a = params.get('action_a', 5.0)
        self._b = params.get('action_b', 5.0)
        self._c1a = params.get('gain_c1_alpha', 1.0)
        self._c2a = params.get('gain_c2_alpha', 2.0)
        self._c1g = params.get('gain_c1_gamma', 0.5)
        self._c2g = params.get('gain_c2_gamma', 1.4)
        self._gamma_mode = str(params.get('gamma_mode', 'circle')).lower()
        self._gamma_center = np.array(
            params.get('gamma_center', [0.0, 0.0]), dtype=float
        )
        self._gamma_radius = params.get('gamma_radius', 1.0)
        self._gamma_speed = params.get('gamma_speed', 0.15)
        self._gamma_heading = params.get('gamma_heading', 0.0)
        self._max_accel = params.get('max_accel', 0.5)

        if self._r <= self._d:
            raise ValueError(
                f"sense_range ({self._r}) must exceed spacing ({self._d}); "
                "otherwise agents at the desired spacing cannot see each other "
                "and the flock never forms."
            )
        if not 0.0 < self._a <= self._b:
            raise ValueError(
                f"action_a ({self._a}) and action_b ({self._b}) must satisfy "
                "0 < a <= b."
            )
        if not 0.0 < self._h < 1.0:
            raise ValueError(f"bump_h ({self._h}) must lie in (0, 1).")

        self._d_alpha = _sigma_norm(np.array([self._d, 0.0]), self._epsilon)
        self._r_alpha = _sigma_norm(np.array([self._r, 0.0]), self._epsilon)
        # Offset that makes the action function vanish at the desired spacing.
        self._c = abs(self._a - self._b) / math.sqrt(4.0 * self._a * self._b)

        self._time = 0.0

    # ── the pairwise action function ──────────────────────────────────────────

    def _phi(self, z: float) -> float:
        """Uneven sigmoid: attractive above 0, repulsive below, bounded both ways."""
        return 0.5 * (
            (self._a + self._b) * _sigma_1(z + self._c) + (self._a - self._b)
        )

    def _phi_alpha(self, z: float) -> float:
        """The action function, zero at the desired spacing and beyond sensing range."""
        return _bump(z / self._r_alpha, self._h) * self._phi(z - self._d_alpha)

    # ── the virtual leader ────────────────────────────────────────────────────

    def _gamma(self) -> Tuple[np.ndarray, np.ndarray]:
        """Position and velocity of the gamma-agent at the current time."""
        if self._gamma_mode == 'static':
            return self._gamma_center.copy(), np.zeros(2)

        if self._gamma_mode == 'line':
            direction = np.array([
                math.cos(self._gamma_heading),
                math.sin(self._gamma_heading),
            ])
            velocity = self._gamma_speed * direction
            return self._gamma_center + velocity * self._time, velocity

        # 'circle' -- the default, because it is the only mode that keeps the
        # flock inside the arena indefinitely.
        omega = self._gamma_speed / self._gamma_radius if self._gamma_radius else 0.0
        angle = omega * self._time
        position = self._gamma_center + self._gamma_radius * np.array([
            math.cos(angle), math.sin(angle),
        ])
        velocity = self._gamma_speed * np.array([
            -math.sin(angle), math.cos(angle),
        ])
        return position, velocity

    # ── control ───────────────────────────────────────────────────────────────

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        self._time += dt
        q_gamma, p_gamma = self._gamma()

        controls = {}

        for drone_id, state in states.items():
            gradient = np.zeros(2)
            matching = np.zeros(2)

            for other_id, other in states.items():
                if other_id == drone_id:
                    continue

                offset = other.position - state.position
                distance = _sigma_norm(offset, self._epsilon)

                # Outside sensing range the bump function is zero, so this
                # neighbour contributes nothing -- the range check is implicit
                # in the maths, and skipping early only saves time.
                if distance > self._r_alpha:
                    continue

                gradient += self._phi_alpha(distance) * _sigma_grad(
                    offset, self._epsilon
                )
                weight = _bump(distance / self._r_alpha, self._h)
                matching += weight * (other.velocity - state.velocity)

            accel = (
                self._c1a * gradient
                + self._c2a * matching
                - self._c1g * (state.position - q_gamma)
                - self._c2g * (state.velocity - p_gamma)
            )

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
                # No setpoint: the lattice is emergent, so there is no reference
                # position to publish. crazyflie_node integrates the command.
            )

        return controls

    def reset(self) -> None:
        self._time = 0.0
