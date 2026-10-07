"""Kuramoto with discrete communication delay only in neighbour theta_j.

History is indexed by controller ticks, not wall-clock receipt times. Before
enough ticks exist, hold each neighbour's first available phase. Current
neighbour phases still define the position formation targets.
"""
from collections import deque
import math
import numpy as np

from .kuramoto import KuramotoFormation
from .registry import register_algorithm


@register_algorithm
class KuramotoPhaseDelay(KuramotoFormation):
    def name(self):
        return 'KuramotoPhaseDelay'

    def configure(self, params, drone_ids):
        if 'phase_delay_ms' in params and 'phase_delay_steps' in params:
            raise ValueError('set phase_delay_ms OR phase_delay_steps, not both')
        self.delay_ms = float(params.get('phase_delay_ms', 0.))
        steps = float(params.get('phase_delay_steps', 0))
        if not math.isfinite(self.delay_ms) or self.delay_ms < 0:
            raise ValueError('phase_delay_ms must be finite and nonnegative')
        if not math.isfinite(steps) or steps < 0 or not steps.is_integer():
            raise ValueError('phase_delay_steps must be a nonnegative integer')
        self._configured_steps = int(steps) if 'phase_delay_steps' in params else None
        super().configure(params, drone_ids)
        self._history = deque()
        self._first_phases = {}
        self._dt = None

    def compute_controls(self, states, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        if self._dt is not None and not math.isclose(dt, self._dt):
            raise ValueError('phase delay requires a fixed controller timestep')
        ratio = self.delay_ms / (1000 * dt)
        if self._configured_steps is None and not math.isclose(ratio, round(ratio), abs_tol=1e-9):
            raise ValueError('phase_delay_ms must be a whole number of controller timesteps')
        delay_steps = self._configured_steps if self._configured_steps is not None else round(ratio)
        self._dt = dt
        for d, agent in self.agents.items():
            if d in states and d not in self._initialized:
                displacement = states[d].position - agent.center
                if self._initialize_from_state and np.linalg.norm(displacement) > 1e-9:
                    agent.phase = math.atan2(displacement[1], displacement[0]) % (2 * math.pi)
                self._initialized.add(d)
        phases = {d: a.phase for d, a in self.agents.items() if d in states}
        for d, phase in phases.items():
            self._first_phases.setdefault(d, phase)
        self._history.append(phases)
        while len(self._history) > delay_steps + 1:
            self._history.popleft()
        delayed = self._history[0] if len(self._history) == delay_steps + 1 else self._first_phases
        return {d: a.step(states[d], {n: states[n] for n in a.neighbors if n in states},
                          {n: phases[n] for n in a.neighbors if n in phases}, dt,
                          coupling_phases={n: delayed[n] for n in a.neighbors
                                           if n in delayed and n in states})
                for d, a in self.agents.items() if d in states}

    def reset(self):
        super().reset()
        self._history.clear()
        self._first_phases.clear()
        self._dt = None
