import math
import unittest
from pathlib import Path
import numpy as np
import yaml
from drone_testbed.algorithms.kuramoto import KuramotoAgent, KuramotoFormation
from drone_testbed.utils.types import DroneState


class KuramotoTests(unittest.TestCase):
    def test_phase_checks_actual_orbital_progress(self):
        for initial_phase in (0., 2 * math.pi - .05):
            for error in (-.3, .3):
                agent = KuramotoAgent('a', {'initial_phases': {'a': initial_phase},
                                            'center': [1., -2.]}, ['a'])
                angle = initial_phase + error
                position = agent.center + agent.radius * np.array([math.cos(angle), math.sin(angle)])
                agent.step(DroneState('a', position), {}, {}, .1)
                expected = (initial_phase + .1 * (agent.omega + agent.tracking_phase_gain
                                                 * math.sin(error))) % (2 * math.pi)
                self.assertAlmostEqual(agent.phase, expected)

    def test_non_neighbor_cannot_affect_local_control(self):
        params = {'adjacency': {'a': ['b']}}
        a = KuramotoAgent('a', params, ['a', 'b', 'c'])
        b = KuramotoAgent('a', params, ['a', 'b', 'c'])
        own = DroneState('a')
        neighbor = DroneState('b', np.array([.4, .2]))
        x = a.step(own, {'b': neighbor}, {'b': .7}, .05)
        y = b.step(own, {'b': neighbor, 'c': DroneState('c', np.array([100., -100.]))},
                   {'b': .7, 'c': -2.}, .05)
        np.testing.assert_array_equal(x.acceleration, y.acceleration)
        self.assertEqual(a.phase, b.phase)

    def test_default_formation_synchronizes_and_rotates(self):
        cfg = yaml.safe_load((Path(__file__).parents[1] / 'config/testbed_kuramoto.yaml').read_text())
        algo = KuramotoFormation()
        ids = [d['id'] for d in cfg['drones']]
        algo.configure(cfg['algorithm']['params'], ids)
        states = {d['id']: DroneState(d['id'], np.array(d['initial_position']), np.zeros(2)) for d in cfg['drones']}
        radii, separations, angles, spacing_errors = [], [], [], []
        dt = cfg['simulation']['dt']
        for k in range(round(120 / dt)):
            controls = algo.compute_controls(states, dt)
            for d, ctrl in controls.items():
                self.assertLessEqual(np.linalg.norm(ctrl.acceleration), .5000001)
                self.assertFalse(ctrl.has_setpoint())
                states[d].position += dt * states[d].velocity + .5 * dt**2 * ctrl.acceleration
                states[d].velocity += dt * ctrl.acceleration
                self.assertLess(np.linalg.norm(states[d].position), 1.)
            separations.append(min(np.linalg.norm(states[a].position - states[b].position)
                                   for i, a in enumerate(ids) for b in ids[i+1:]))
            if k * dt > 80:
                radii.append(np.linalg.norm(states['drone1'].position))
                actual = np.array([np.arctan2(states[d].position[1], states[d].position[0]) for d in ids])
                angles.append(actual[0])
                gaps = np.roll(actual, -1) - actual - 2*np.pi/len(ids)
                spacing_errors.append(np.max(np.abs(np.arctan2(np.sin(gaps), np.cos(gaps)))))
        self.assertGreater(min(separations), .5)
        self.assertLess(max(abs(r - .65) for r in radii), .03)
        self.assertGreater(np.unwrap(angles)[-1] - np.unwrap(angles)[0], 4*np.pi)
        self.assertLess(max(spacing_errors), .03)
        phases = [a.phase - a.phase_offsets[d] for d, a in algo.agents.items()]
        self.assertGreater(abs(sum(np.exp(1j * p) for p in phases) / len(phases)), .999)
        algo.reset()
        self.assertAlmostEqual(algo.agents['drone1'].phase, 0.)

    def test_rotating_equilibrium_and_wraparound(self):
        ids = ['a', 'b', 'c', 'd']
        for rotation in (0., 2*np.pi - .01):
            params = {'initial_phases': {d: rotation + i*np.pi/2 for i, d in enumerate(ids)}}
            algo = KuramotoFormation()
            algo.configure(params, ids)
            states = {}
            for d, agent in algo.agents.items():
                radial = np.array([np.cos(agent.phase), np.sin(agent.phase)])
                states[d] = DroneState(d, agent.radius * radial,
                                      agent.radius * agent.omega * np.array([-radial[1], radial[0]]))
            controls = algo.compute_controls(states, .05)
            for d, ctrl in controls.items():
                np.testing.assert_allclose(ctrl.acceleration, -.35**2 * states[d].position, atol=1e-12)

    def test_measured_initialization_and_reset(self):
        algo = KuramotoFormation()
        algo.configure({'center': [1., 2.]}, ['a'])
        state = DroneState('a', np.array([1., 2.65]))
        algo.compute_controls({'a': state}, .1)
        self.assertAlmostEqual(algo.get_phases()['a'], np.pi / 2 + .035)
        algo.reset()
        self.assertEqual(algo.get_phases(), {})
        state.position = np.array([.35, 2.])
        algo.compute_controls({'a': state}, .1)
        self.assertAlmostEqual(algo.get_phases()['a'], np.pi + .035)
        algo.configure({'initialize_from_state': False}, ['a'])
        algo.compute_controls({'a': DroneState('a')}, .1)
        self.assertAlmostEqual(algo.get_phases()['a'], .035)

    def test_missing_neighbor_phase_is_not_coupled(self):
        algo = KuramotoFormation()
        algo.configure({'initialize_from_state': False,
                        'initial_phases': {'b': 1.}}, ['a', 'b'])
        algo.compute_controls({'a': DroneState('a')}, .1)
        self.assertAlmostEqual(algo.agents['a'].phase, .035)

    def test_weighted_adjacency(self):
        agent = KuramotoAgent('a', {'adjacency': {'a': {'b': 2.}},
                                    'max_accel': 100.}, ['a', 'b'])
        agent.step(DroneState('a'), {}, {'b': np.pi + .2}, .1)
        self.assertAlmostEqual(agent.phase, .1 * (.35 + .5 * np.sin(.2)))
        for weight in (-1., np.nan, np.inf):
            with self.assertRaises(ValueError):
                KuramotoAgent('a', {'adjacency': {'a': {'b': weight}}}, ['a', 'b'])

    def test_missing_neighbors_and_validation(self):
        agent = KuramotoAgent('a', {}, ['a', 'b'])
        out = agent.step(DroneState('a'), {}, {}, .1)
        self.assertTrue(np.all(np.isfinite(out.acceleration)))
        self.assertAlmostEqual(agent.phase, .035)
        for params in [{'radius': 0}, {'max_accel': 0},
                       {'adjacency': {'a': ['unknown']}}, {'omega': math.nan},
                       {'tracking_phase_gain': -1}, {'tracking_phase_gain': math.nan}]:
            with self.assertRaises(ValueError):
                KuramotoAgent('a', params, ['a', 'b'])


from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from drone_testbed.algorithm_manager import AlgorithmManagerNode
from drone_testbed.algorithms.kuramoto import KuramotoFormation
from drone_testbed.utils.controller_launch import controller_nodes
from drone_testbed.utils.types import DroneState


def test_launch_uses_same_manager_for_both_algorithms():
    for name in ('KuramotoFormation', 'TrochoidalConsensus'):
        with patch('drone_testbed.utils.controller_launch.Node') as node:
            result = controller_nodes({'algorithm': {'name': name}}, '/config.yaml', 12.)
            assert result == [node.return_value]
            node.assert_called_once_with(
                package='drone_testbed', executable='algorithm_manager',
                name='algorithm_manager', parameters=[{
                    'config_file': '/config.yaml', 'auto_start_delay': 12.}],
                output='screen')


def test_manager_waits_then_publishes_acceleration_and_phase():
    algo = KuramotoFormation()
    algo.configure({}, ['a', 'b'])
    manager = SimpleNamespace(
        _started=False, _drone_ids=['a', 'b'], _states={},
        _algorithm=algo, _dt=.1,
        _cmd_pubs={d: Mock() for d in ['a', 'b']},
        _pos_pubs={d: Mock() for d in ['a', 'b']},
        _phase_pubs={d: Mock() for d in ['a', 'b']},
    )
    AlgorithmManagerNode._control_loop(manager)
    manager._started = True
    manager._states['a'] = DroneState('a', np.array([.65, 0.]))
    AlgorithmManagerNode._control_loop(manager)
    manager._cmd_pubs['a'].publish.assert_not_called()
    assert algo.get_phases() == {}
    manager._states['b'] = DroneState('b', np.array([-.65, 0.]))
    AlgorithmManagerNode._control_loop(manager)
    for d in ['a', 'b']:
        command = manager._cmd_pubs[d].publish.call_args.args[0]
        assert len(command.data) == 2
        assert np.linalg.norm(command.data) <= .5
        phase = manager._phase_pubs[d].publish.call_args.args[0]
        assert phase.data == algo.get_phases()[d]
        manager._pos_pubs[d].publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
