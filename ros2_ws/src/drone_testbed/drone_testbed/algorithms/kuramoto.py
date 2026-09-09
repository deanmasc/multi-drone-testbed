"""Kuramoto-coupled rotating ring formation for double-integrator agents.

Each local controller owns one phase and consumes only self/neighbor state.
The BaseAlgorithm adapter is for standalone numerical simulation; ROS launches
use separate local controllers, never this all-agent adapter.
"""
import math
import numpy as np

from .base_algorithm import BaseAlgorithm
from .registry import register_algorithm
from drone_testbed.utils.types import ControlOutput


class KuramotoAgent:
    def __init__(self, drone_id, params, drone_ids):
        if not drone_ids or len(set(drone_ids)) != len(drone_ids):
            raise ValueError('drone IDs must be nonempty and unique')
        self.drone_id = drone_id
        index = drone_ids.index(drone_id)
        graph = params.get('adjacency')
        self.neighbors = list(graph.get(drone_id, [])) if graph is not None else list(dict.fromkeys(
            d for d in (drone_ids[(index - 1) % len(drone_ids)],
                        drone_ids[(index + 1) % len(drone_ids)]) if d != drone_id))
        if len(set(self.neighbors)) != len(self.neighbors) or any(
                d == drone_id or d not in drone_ids for d in self.neighbors):
            raise ValueError('neighbors must be unique known IDs excluding self')
        # Phase is the physical orbital angle. Subtract the assigned polygon
        # offset before coupling so synchronization preserves equal spacing.
        self.phase_offsets = {d: 2 * math.pi * i / len(drone_ids)
                              for i, d in enumerate(drone_ids)}
        self.center = np.asarray(params.get('center', [0., 0.]), dtype=float)
        if self.center.shape != (2,) or not np.all(np.isfinite(self.center)):
            raise ValueError('center must contain two finite coordinates')
        for key, default in [('radius', .65), ('omega', .35),
                             ('phase_gain', .25), ('tracking_phase_gain', .4), ('position_gain', 1.),
                             ('formation_gain', .8), ('velocity_gain', 2.), ('max_accel', .5)]:
            value = float(params.get(key, default))
            if not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
            setattr(self, key, value)
        if self.radius <= 0 or self.max_accel <= 0 or self.velocity_gain <= 0:
            raise ValueError('require radius > 0, max_accel > 0, velocity_gain > 0')
        if min(self.phase_gain, self.tracking_phase_gain, self.position_gain, self.formation_gain) < 0:
            raise ValueError('coupling and position gains must be nonnegative')
        self.initial_phase = float(params.get('initial_phases', {}).get(drone_id, self.phase_offsets[drone_id]))
        if not math.isfinite(self.initial_phase):
            raise ValueError('initial phases must be finite')
        self.reset()

    def reset(self):
        self.phase = self.initial_phase % (2 * math.pi)

    def step(self, state, neighbor_states, neighbor_phases, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        own_offset = self.phase_offsets[self.drone_id]
        phase_rate = self.omega + self.phase_gain * sum(
            math.sin((neighbor_phases[d] - self.phase_offsets[d])
                     - (self.phase - own_offset))
            for d in self.neighbors if d in neighbor_phases)
        # Check the physical orbital angle on every step. A lagging drone slows
        # its oscillator, feeding the motion error back into neighbor coupling.
        # Near the center the angle is undefined; smoothly suppress this check.
        displacement = state.position - self.center
        distance = float(np.linalg.norm(displacement))
        if distance > 1e-9:
            actual_phase = math.atan2(displacement[1], displacement[0])
            phase_rate += (self.tracking_phase_gain * min(distance / self.radius, 1.)
                           * math.sin(actual_phase - self.phase))
        radial = np.array([math.cos(self.phase), math.sin(self.phase)])
        tangent = np.array([-radial[1], radial[0]])
        target = self.center + self.radius * radial
        target_velocity = self.radius * phase_rate * tangent
        relative_error = np.zeros(2)
        for d in self.neighbors:
            if d in neighbor_states and d in neighbor_phases:
                angle = neighbor_phases[d]
                neighbor_target = self.center + self.radius * np.array([
                    math.cos(angle), math.sin(angle)])
                desired_offset = neighbor_target - target
                relative_error += neighbor_states[d].position - state.position - desired_offset
        # Centripetal feedforward removes the steady circular tracking bias.
        # Tangential acceleration during phase convergence is handled by feedback.
        accel = (-self.radius * phase_rate**2 * radial
                 + self.position_gain * (target - state.position)
                 + self.formation_gain * relative_error
                 + self.velocity_gain * (target_velocity - state.velocity))
        norm = np.linalg.norm(accel)
        if norm > self.max_accel:
            accel *= self.max_accel / norm
        self.phase = (self.phase + dt * phase_rate) % (2 * math.pi)
        return ControlOutput(self.drone_id, accel)


@register_algorithm
class KuramotoFormation(BaseAlgorithm):
    """Synchronous simulation adapter around independent local agents."""
    def name(self):
        return 'KuramotoFormation'

    def configure(self, params, drone_ids):
        self.agents = {d: KuramotoAgent(d, params, drone_ids) for d in drone_ids}

    def compute_controls(self, states, dt):
        phases = {d: a.phase for d, a in self.agents.items()}
        return {d: a.step(states[d], {n: states[n] for n in a.neighbors if n in states},
                          {n: phases[n] for n in a.neighbors}, dt)
                for d, a in self.agents.items() if d in states}

    def reset(self):
        for agent in self.agents.values():
            agent.reset()
