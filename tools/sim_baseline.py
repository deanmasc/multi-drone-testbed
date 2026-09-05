#!/usr/bin/env python3
"""Produce a metrics record from simulation, in the same format as a flight.

The hardware recorder (tools/metrics_recorder.py) needs something to be compared
against, and the honest comparison is the *same config* with the physical layer
removed. Any difference between the two files is then gap B -- estimation noise,
latency, motor limits, aerodynamics -- because simulation has already accounted
for everything in gap A.

This runs headless and needs no ROS, so baselines can be generated on a laptop
before a lab session. It uses the repo's own dynamics (dynamics/double_
integrator.py, an exact zero-order-hold step) rather than a hand-rolled
integrator, because a coarse integrator invents decay that is easily mistaken
for a real result -- see docs/PROJECT_AIM.md section 6a.

    python3 tools/sim_baseline.py --config <path> [--duration 90]

Writes logs/<algorithm>_sim_<YYYYmmdd_HHMMSS>.txt
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'ros2_ws', 'src', 'drone_testbed')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import metrics_recorder as M                                    # noqa: E402
from drone_testbed.utils.types import DroneState                # noqa: E402
from drone_testbed.dynamics.double_integrator import step       # noqa: E402
from drone_testbed.algorithms.registry import get_algorithm     # noqa: E402
import drone_testbed.algorithms                                 # noqa: E402,F401


def run(cfg, duration, rate=None):
    ids = [d['id'] for d in cfg['drones']]
    acfg = cfg.get('algorithm', {})
    params = acfg.get('params', {}) or {}
    rate = rate or float(cfg['simulation'].get('control_rate', 10.0))
    dt = 1.0 / rate

    algo = get_algorithm(acfg['name'])
    algo.configure(dict(params), list(ids))
    metrics = M.make_metrics(acfg['name'], ids, params)

    state = {d['id']: np.array(list(d['initial_position']) +
                               list(d.get('initial_velocity', [0.0, 0.0])),
                               dtype=float)
             for d in cfg['drones']}

    ts, rows, pos_hist = [], [], []
    t = 0.0
    while t < duration:
        ds = {i: DroneState(i, state[i][:2].copy(), state[i][2:].copy())
              for i in ids}
        out = algo.compute_controls(ds, dt)
        for i in ids:
            state[i] = step(state[i], out[i].acceleration, dt)

        pos = np.array([state[i][:2] for i in ids])
        vel = np.array([state[i][2:] for i in ids])
        row = metrics.row(t, pos, vel)
        if row is not None:
            ts.append(t)
            rows.append(row)
            pos_hist.append(pos.copy())
        t += dt

    return metrics, np.array(ts), np.array(rows), pos_hist, rate


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--config', required=True)
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--rate', type=float, default=None,
                    help="control rate; default is the config's control_rate")
    ap.add_argument('--out-dir', default='logs')
    a = ap.parse_args()

    with open(a.config) as f:
        cfg = yaml.safe_load(f)

    metrics, ts, data, pos_hist, rate = run(cfg, a.duration, a.rate)
    algo = cfg['algorithm']['name']
    ids = [d['id'] for d in cfg['drones']]

    os.makedirs(a.out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(a.out_dir, f'{algo.lower()}_sim_{stamp}.txt')

    with open(path, 'w') as fh:
        M.write_header(fh, cfg, algo, ids, metrics.columns(), rate)
        for row in data:
            fh.write(M.format_row(row) + '\n')

        lines = ['', '=' * 74,
                 f'SUMMARY -- {algo}  (SIMULATION BASELINE, no hardware)',
                 '=' * 74, '',
                 f'  source config  {a.config}',
                 f'  control rate   {rate} Hz',
                 f'  samples {len(ts)} over {ts[-1] - ts[0]:.1f} s',
                 '']
        lines += metrics.summarise(ts, data, pos_hist)
        fh.write('\n'.join('# ' + ln if ln else '#' for ln in lines) + '\n')

    print('\n'.join(lines))
    print(f'\n[sim_baseline] written to {path}\n')


if __name__ == '__main__':
    main()
