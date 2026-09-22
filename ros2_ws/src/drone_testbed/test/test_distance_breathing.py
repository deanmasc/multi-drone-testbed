"""Run without ROS: python3 -m unittest discover -s ros2_ws/src/drone_testbed/test -p test_distance_breathing.py"""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/drone_testbed'))
import metrics_recorder as M
from drone_testbed.algorithms.distance_formation import (
    BreathingProfile, DistanceFormation, formation_spec)
from drone_testbed.utils.types import DroneState


class BreathingTests(unittest.TestCase):
    def setUp(self):
        self.ids = [f'drone{i}' for i in range(1, 7)]
        self.params = dict(side_length=.7, breathing_amplitude=.1,
                           breathing_period=20., breathing_start_delay=5.,
                           breathing_ramp_duration=2., max_accel=100.)

    def algorithm(self, params=None):
        a = DistanceFormation()
        with contextlib.redirect_stdout(io.StringIO()):
            a.configure(self.params if params is None else params, self.ids)
        return a

    def states(self, pos, vel=None):
        vel = np.zeros_like(pos) if vel is None else vel
        return {i: DroneState(i, p.copy(), v.copy())
                for i, p, v in zip(self.ids, pos, vel)}

    def test_profile_extrema_and_smooth_start(self):
        p = BreathingProfile(self.params)
        self.assertEqual(p.scale(0), 1)
        self.assertEqual(p.scale(5), 1)
        self.assertAlmostEqual(p.scale(10), 1.1)
        self.assertAlmostEqual(p.scale(20), .9)
        self.assertAlmostEqual(p.scale(30), 1.1)
        self.assertLess(abs(p.scale(5 + 1e-4) - 1), 1e-10)

    def test_invalid_parameters(self):
        for key, values in [('breathing_amplitude', [-.1, 1., np.nan]),
                            ('breathing_period', [0., -1., np.inf]),
                            ('breathing_start_delay', [-1., np.nan]),
                            ('breathing_ramp_duration', [0., -1., np.inf])]:
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.algorithm(dict(self.params, **{key: value}))

    def test_zero_amplitude_preserves_static_law(self):
        params = dict(self.params, breathing_amplitude=0, gain_kp=.3,
                      gain_kv=1.2, max_accel=.5)
        targets, edges, distances = formation_spec(params, self.ids)
        rng = np.random.default_rng(17)
        pos = targets + rng.normal(0, .1, targets.shape)
        vel = rng.normal(0, .05, targets.shape)
        expected = -1.2 * vel
        for (i, j), d in zip(edges, distances):
            z = pos[i] - pos[j]
            force = -.3 * (z @ z - d*d) * z
            expected[i] += force
            expected[j] -= force
        a = self.algorithm(params)
        for dt in (.1, 50., .1):
            out = a.compute_controls(self.states(pos, vel), dt)
            np.testing.assert_allclose([out[i].acceleration for i in self.ids],
                                       np.clip(expected, -.5, .5), atol=1e-14)

    def test_scaled_shape_and_anchors_share_same_geometry(self):
        params = dict(self.params, formation_center=[.2, -.1],
                      anchor=['drone1', 'drone3'])
        a = self.algorithm(params)
        nominal, _, _ = formation_spec(params, self.ids)
        a.compute_controls(self.states(nominal), 10.)
        center = np.array(params['formation_center'])
        pos = center + 1.1 * (nominal - center)
        out = a.compute_controls(self.states(pos), .1)
        np.testing.assert_allclose([out[i].acceleration for i in self.ids], 0, atol=1e-14)
        self.assertEqual(a.reference(), (10., 1.1))
        a.reset()
        a.compute_controls(self.states(nominal), .1)
        self.assertEqual(a.reference(), (0., 1.))

    def test_force_cancellation_without_anchors_or_clipping(self):
        a = self.algorithm()
        nominal, _, _ = formation_spec(self.params, self.ids)
        a.compute_controls(self.states(nominal), 10.)
        pos = nominal + np.random.default_rng(4).normal(0, .03, nominal.shape)
        out = a.compute_controls(self.states(pos), .1)
        np.testing.assert_allclose(sum(x.acceleration for x in out.values()), 0, atol=1e-14)

    def metric(self):
        return M.DistanceFormationMetrics(self.ids, self.params)

    def test_metric_uses_published_reference_not_recording_clock(self):
        m = self.metric()
        m.set_reference(10., 1.1)
        row = dict(zip(m.columns(), m.row(999., m.targets * 1.1, np.zeros((6, 2)))))
        self.assertEqual(row['reference_time'], 10.)
        self.assertEqual(row['reference_valid'], 1.)
        self.assertAlmostEqual(row['edge_rms'], 0.)
        self.assertAlmostEqual(row['shape_err'], 0.)
        self.assertAlmostEqual(row['actual_scale'], 1.1)
        self.assertEqual(row['t'], 999.)

    def test_missing_stale_and_mismatched_references_are_not_scored(self):
        m = self.metric()
        for ref in [(np.nan, np.nan, 0., 0.), (10., 1.1, .6, 0.),
                    (10., 1.1, 0., .6), (10., .9, 0., 0.)]:
            m.set_reference(*ref)
            values = m.full_row(0., m.targets, np.zeros((6, 2)))
            self.assertEqual(len(values), len(m.all_columns()))
            row = dict(zip(m.all_columns(), values))
            self.assertEqual(row['reference_valid'], 0.)
            self.assertTrue(np.isnan(row['edge_rms']))
            self.assertTrue(np.isfinite(row['x_drone1']))

    def synthetic_record(self, m):
        t = np.arange(7., 68., .1)
        rows = []
        for rt in t:
            m.set_reference(rt, m.breathing.scale(rt))
            # A known 2 s lag and half the commanded amplitude.
            actual = 1 + .05 * np.sin(2*np.pi*(rt - 5 - 2)/20)
            rows.append(m.full_row(rt + 900, m.targets*actual, np.zeros((6, 2))))
        return t + 900, np.asarray(rows)

    def test_tracking_summary_recovers_lag_and_amplitude(self):
        m = self.metric()
        t, rows = self.synthetic_record(m)
        summary = '\n'.join(m.summarise(t, rows, []))
        self.assertIn('0.500', summary)
        self.assertIn('2.000 s', summary)
        self.assertNotIn('NEVER settled', summary)
        self.assertNotIn('must not increase', summary)
        rows[100, m.columns().index('reference_valid')] = 0
        self.assertIn('requires a continuous full cycle', '\n'.join(m.summarise(t, rows, [])))

    def test_offline_reanalysis_keeps_controller_phase(self):
        m = self.metric()
        t, rows = self.synthetic_record(m)
        cfg = {'algorithm': {'name': 'DistanceFormation', 'params': self.params},
               'drones': [{'id': i} for i in self.ids]}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'record.txt'
            np.savetxt(path, rows)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                M.reanalyse(path, cfg, t[0], t[-1])
            self.assertIn('2.000 s', output.getvalue())


if __name__ == '__main__':
    unittest.main()
