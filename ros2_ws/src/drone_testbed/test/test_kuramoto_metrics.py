import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('metrics_recorder',
    Path(__file__).resolve().parents[4] / 'tools/metrics_recorder.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MetricsTests(unittest.TestCase):
    def setUp(self):
        self.m = module.make_metrics('KuramotoFormation', ['a', 'b'], {})
        self.pos = np.array([[.65, 0.], [-.65, 0.]])
        self.vel = np.zeros((2, 2))

    def row(self, phases=(0., 0.), ages=(0., 0.), t=0., active=True):
        return self.m.row(t, self.pos, self.vel, phases, ages, [0., 0.], active)

    def test_known_phases_and_tracking(self):
        row = dict(zip(self.m.columns(), self.row()))
        self.assertEqual(row['order_R'], 1.)
        self.assertAlmostEqual(row['tracking_rms'], 0.)
        self.assertAlmostEqual(row['d_min'], 1.3)
        row = dict(zip(self.m.columns(), self.row((0., np.pi))))
        self.assertAlmostEqual(row['order_R'], 0.)
        row = dict(zip(self.m.columns(), self.row((.01, 2*np.pi-.01))))
        self.assertAlmostEqual(row['phase_diff_a_b'], .02)

    def test_missing_stale_and_skewed_phases(self):
        for phases, ages in [((0., np.nan), (0., 0.)), ((0., 0.), (0., .6)),
                             ((0., 0.), (0., .2))]:
            row = dict(zip(self.m.columns(), self.row(phases, ages)))
            self.assertEqual(row['phase_valid'], 0.)
            self.assertTrue(np.isnan(row['order_R']))
        row = dict(zip(self.m.columns(), self.m.row(0., self.pos, self.vel,
                           [0., 0.], [0., 0.], [0., 1.], True)))
        self.assertTrue(np.isnan(row['tracking_rms']))
        self.assertTrue(np.isnan(row['d_min']))

    def test_hold_requires_active_contiguous_valid_samples(self):
        t = np.arange(0., 6.1, .1)
        data = np.array([self.row(t=x) for x in t])
        summary = '\n'.join(self.m.summarise(t, data, []))
        self.assertIn('t = 0.000s', summary)
        data[30] = self.row(phases=(np.nan, 0.), t=3.)
        self.assertIn('not observed', '\n'.join(self.m.summarise(t, data, [])))
        data = np.array([self.row(t=x, active=False) for x in t])
        self.assertIn('not observed', '\n'.join(self.m.summarise(t, data, [])))
        t = np.r_[np.arange(0., 3., .1), np.arange(4., 7., .1)]
        data = np.array([self.row(t=x) for x in t])
        self.assertIn('not observed', '\n'.join(self.m.summarise(t, data, [])))

    def test_other_algorithms_keep_existing_metrics(self):
        self.assertIsInstance(module.make_metrics('Flocking', ['a', 'b'], {}), module.FlockingMetrics)
        self.assertIsInstance(module.make_metrics('TrochoidalConsensus', ['a', 'b'], {}), module.TrochoidalMetrics)


if __name__ == '__main__':
    unittest.main()
