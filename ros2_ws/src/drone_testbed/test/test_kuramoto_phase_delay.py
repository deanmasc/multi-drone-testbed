import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from drone_testbed.algorithms.kuramoto import KuramotoFormation
from drone_testbed.algorithms.kuramoto_phase_delay import KuramotoPhaseDelay
from drone_testbed.algorithms.registry import get_algorithm
from drone_testbed.utils.types import DroneState


def make_algo(params=None):
    algo = KuramotoPhaseDelay()
    algo.configure({'initialize_from_state': False, 'tracking_phase_gain': 0.,
                    'velocity_gain': 1., 'initial_phases': {'b': math.pi + .4},
                    **(params or {})}, ['a', 'b'])
    return algo


def states():
    return {d: DroneState(d, np.array([.3, .2])) for d in ['a', 'b']}


def test_zero_delay_matches_original():
    delayed = make_algo()
    original = KuramotoFormation()
    original.configure({'initialize_from_state': False, 'tracking_phase_gain': 0.,
                        'velocity_gain': 1., 'initial_phases': {'b': math.pi + .4}}, ['a', 'b'])
    for _ in range(30):
        x = delayed.compute_controls(states(), .05)
        y = original.compute_controls(states(), .05)
        assert delayed.get_phases() == original.get_phases()
        for d in x:
            np.testing.assert_array_equal(x[d].acceleration, y[d].acceleration)


@pytest.mark.parametrize('params,lag', [({'phase_delay_ms': 50}, 1),
                                       ({'phase_delay_ms': 100}, 2),
                                       ({'phase_delay_ms': 200}, 4),
                                       ({'phase_delay_steps': 3}, 3)])
def test_uses_exact_past_neighbour_phase_with_current_own_phase(params, lag):
    algo = make_algo(params)
    snapshots = []
    for tick in range(12):
        own = algo.agents['a'].phase
        snapshots.append(algo.agents['b'].phase)
        theta_j = snapshots[max(0, tick - lag)]
        expected = (own + .05 * (.35 + .25 * math.sin(theta_j - math.pi - own))) % (2 * math.pi)
        algo.compute_controls(states(), .05)
        assert algo.agents['a'].phase == pytest.approx(expected)
        assert len(algo._history) <= lag + 1


def test_only_coupling_is_delayed_not_formation_targets():
    algo = make_algo({'phase_delay_steps': 2, 'phase_gain': 0.})
    original = KuramotoFormation()
    original.configure({'initialize_from_state': False, 'tracking_phase_gain': 0.,
                        'velocity_gain': 1., 'phase_gain': 0.,
                        'initial_phases': {'b': math.pi + .4}}, ['a', 'b'])
    for _ in range(10):
        x = algo.compute_controls(states(), .05)
        y = original.compute_controls(states(), .05)
        for d in x:
            np.testing.assert_array_equal(x[d].acceleration, y[d].acceleration)


def test_reset_clears_delay_history_and_reinitializes():
    algo = make_algo({'phase_delay_ms': 100})
    first = algo.compute_controls(states(), .05)
    for _ in range(5):
        algo.compute_controls(states(), .05)
    algo.reset()
    assert algo.get_phases() == {}
    assert not algo._history
    assert not algo._first_phases
    again = algo.compute_controls(states(), .05)
    for d in first:
        np.testing.assert_array_equal(first[d].acceleration, again[d].acceleration)


@pytest.mark.parametrize('params', [{'phase_delay_ms': -1}, {'phase_delay_ms': math.nan},
                                   {'phase_delay_steps': 1.5}, {'phase_delay_steps': -1},
                                   {'phase_delay_ms': 0, 'phase_delay_steps': 0}])
def test_invalid_delay_rejected(params):
    with pytest.raises(ValueError):
        make_algo(params)


def test_unrepresentable_delay_and_changing_dt_rejected():
    algo = make_algo({'phase_delay_ms': 50})
    with pytest.raises(ValueError, match='whole number'):
        algo.compute_controls(states(), .1)
    algo.compute_controls(states(), .05)
    with pytest.raises(ValueError, match='fixed'):
        algo.compute_controls(states(), .1)


def test_test_configs_registered_and_executable():
    root = Path(__file__).parents[1] / 'config'
    for delay in (0, 50, 100, 200):
        cfg = yaml.safe_load((root / f'testbed_kuramoto_delay_{delay:03d}ms.yaml').read_text())
        algo = get_algorithm(cfg['algorithm']['name'])
        ids = [d['id'] for d in cfg['drones']]
        algo.configure(cfg['algorithm']['params'], ids)
        assert cfg['algorithm']['params']['velocity_gain'] == 1.
        initial = {d['id']: DroneState(d['id'], np.array(d['initial_position'])) for d in cfg['drones']}
        for _ in range(10):
            outputs = algo.compute_controls(initial, 1 / cfg['simulation']['control_rate'])
            assert all(np.all(np.isfinite(c.acceleration)) for c in outputs.values())
