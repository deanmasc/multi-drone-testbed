import math
import unittest
from pathlib import Path
import numpy as np
import yaml
from drone_testbed.algorithms.kuramoto import KuramotoAgent, KuramotoFormation
from drone_testbed.utils.types import DroneState


class KuramotoTests(unittest.TestCase):
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

    def test_default_formation_synchronizes_and_breathes(self):
        cfg = yaml.safe_load((Path(__file__).parents[1] / 'config/testbed_kuramoto.yaml').read_text())
        algo = KuramotoFormation()
        ids = [d['id'] for d in cfg['drones']]
        algo.configure(cfg['algorithm']['params'], ids)
        states = {d['id']: DroneState(d['id'], np.array(d['initial_position']), np.zeros(2)) for d in cfg['drones']}
        radii, separations = [], []
        for k in range(2400):
            controls = algo.compute_controls(states, .05)
            for d, ctrl in controls.items():
                self.assertLessEqual(np.linalg.norm(ctrl.acceleration), .5000001)
                states[d].position += .05 * states[d].velocity + .5 * .05**2 * ctrl.acceleration
                states[d].velocity += .05 * ctrl.acceleration
                self.assertLess(np.linalg.norm(states[d].position), 1.)
            separations.append(min(np.linalg.norm(states[a].position - states[b].position)
                                   for i, a in enumerate(ids) for b in ids[i+1:]))
            if k > 1600:
                radii.append(np.linalg.norm(states['drone1'].position))
        self.assertGreater(min(separations), .6)
        self.assertGreater(max(radii) - min(radii), .25)
        phases = [a.phase for a in algo.agents.values()]
        self.assertGreater(abs(sum(np.exp(1j * p) for p in phases) / len(phases)), .999)
        algo.reset()
        self.assertAlmostEqual(algo.agents['drone1'].phase, 0.)

    def test_missing_neighbors_and_validation(self):
        agent = KuramotoAgent('a', {}, ['a', 'b'])
        out = agent.step(DroneState('a'), {}, {}, .1)
        self.assertTrue(np.all(np.isfinite(out.acceleration)))
        self.assertAlmostEqual(agent.phase, .05)
        for params in [{'radius': .1, 'amplitude': .2}, {'max_accel': 0},
                       {'adjacency': {'a': ['unknown']}}, {'omega': math.nan}]:
            with self.assertRaises(ValueError):
                KuramotoAgent('a', params, ['a', 'b'])


if __name__ == '__main__':
    unittest.main()
