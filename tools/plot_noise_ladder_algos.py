#!/usr/bin/env python3
"""Plots for the flocking and coverage measurement-noise ladders (29 Sep 2026).

Two ladders, same lever as the trochoidal one on 23 Sep: identical flights
except for the artificial Gaussian noise added to drone1's VICON position.
Flocking got 0 (control), 2, 5, 10 mm per axis; coverage also got a 20 mm rung,
because coverage's own gain is the smallest of the three algorithms and the
first pass suggested 10 mm would not be enough to move it.

    python3 tools/plot_noise_ladder_algos.py --algo flocking
    python3 tools/plot_noise_ladder_algos.py --algo coverage

Only drone1 is real; drone2-4 are simulated double integrators inside the same
node, so the noise enters the law through one agent and reaches the rest only
through the coupling terms.

Two benchmarks are reported side by side, and the difference between them is
the point of these figures:

  1 DEVIATION FROM THE DESIGNED PATTERN -- the same config replayed in the
    noiseless simulator from where the fleet actually was when the algorithm
    engaged. This is the sim-to-hardware gap, the same quantity the trochoidal
    ladder reports, so the three ladders are comparable.

  2 THE ALGORITHM'S OWN SUCCESS METRICS -- lattice error, velocity spread,
    smallest pair gap and connectivity for flocking; centroid distance and
    locational cost H for coverage. These are the numbers the source papers
    plot to show the theorem held.

Unlike the trochoidal case, where the published metric IS the deviation, here
the two disagree: the flights get measurably worse while most of the published
metrics report no change at all.

WHICH CHANNEL EACH NUMBER COMES FROM. metrics_recorder takes x and y from
/<drone>/state, which for a real drone is mocap_state_node's output -- the
NOISY reading, the same one the algorithm saw -- and takes z and orientation
from /poses, which the injection never touches. So anything differentiated
from the recorded position is measuring the sensor as well as the aircraft,
and white noise differentiates badly: a first difference at 10 Hz multiplies
it by 14, a second difference by 245. Measured against a null model (the
control run's own trajectory re-measured through each rung's noisier sensor,
200 draws) that leaves:

  deviation   1% inflated at the top rung            -- keep
  oscillation 2-4x the null for coverage             -- keep for coverage
              at the null for flocking               -- artefact, not plotted
  flown speed at or near the null everywhere         -- dropped
  achieved a  at or near the null everywhere         -- dropped, use tilt
  tilt, z     from /poses, never noisy               -- the clean witness

The two dropped panels were in the first version of these figures; the ratios
they showed (speed x2.0, acceleration x12) were mostly the differentiator
amplifying the injected noise, not the aircraft flying harder.
"""

import argparse
import copy
import json
import os
import re
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.lines import Line2D                             # noqa: E402
from scipy.signal import butter, sosfiltfilt                    # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyse_trochoidal as A                                  # noqa: E402
import sim_baseline as S                                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed', 'config')
LOGS = os.path.join(ROOT, 'logs', 'hw')
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed'))
from drone_testbed.algorithms.registry import get_algorithm      # noqa: E402
from sim_baseline import DroneState                              # noqa: E402

REAL = ('drone1',)
DT = 0.1                  # the recorder's row interval
WIN_SKIP = 2.0            # start the window this long after the algorithm engages
SHAKE_BAND = (0.6, 1.5)   # the k*tau brake loop, ~1.1 s period
TAU = 0.28                # measured sense-to-move delay, s

INK = '#1f2328'
MUTED = '#6b7280'
GRID = '#e5e7eb'
DESIGN = '#9ca3af'
FLAT = '#c4b8a8'          # the "no trend" bars, so they read as a set

# Cool ramp, light to dark with the noise level, shared with the trochoidal
# ladder: every noise figure in the corpus uses the same visual language, and
# the rungs are an ordered magnitude rather than categories. The 5-rung ramp is
# evenly stepped in OKLab lightness between the same two endpoints, so the
# coverage figure keeps the ends of the trochoidal one.
RAMP4 = ['#a3c9e6', '#5b9bd0', '#2a6ca8', '#0d3b63']
RAMP5 = ['#a3c9e6', '#7ca3c4', '#577fa3', '#335c82', '#0d3b63']

# The flocking window stops at engage + 2 + 124 s = ~138 s. The 2 mm record has
# a 0.9 s physical excursion at t = 144.0-144.8 s -- drone1 jumps from r = 0.34
# to r = 1.05 m and its altitude drops to 0.83 m, which is a tracking loss on a
# tiring battery, not anything the algorithm did. Over the full 148 s window
# used for the trochoidal ladder that single event alone takes the 2 mm rung
# from 4.8 to 21.0 cm RMS and inverts the ladder. The window is therefore cut
# to end before it, in every rung at the same offset, and the excursion is
# reported in the table rather than hidden.
#
# The coverage window is 112 s because the 2-20 mm flights ran 120 s. Only the
# 0 mm control ran 180 s (it was flown before the 180 s duration was shown to
# be unnecessary), so the control is truncated to the same window as the rest.
SPEC = {
    # --- 7 Oct ladders -----------------------------------------------------
    # Both sit at k*tau 0.39, well below the 0.67-0.81 oscillation threshold,
    # so the noise is the only thing acting. Each rung differs from its control
    # in mocap_noise and nothing else -- verified by diffing the parsed params,
    # not by reading the files. Distance formation uses the STATIC hybrid
    # config, not the breathing one: breathing would be a second variable, and
    # the 6 Oct k04 flights were static anyway (the install space predated the
    # breathing code, PROJECT_AIM 16f).
    'kuramoto': dict(
        title='kuramoto',
        win=130.0,
        ramp=RAMP4,
        pattern='kuramoto ring',
        runs=[('0', 'REPLACE_kuramoto_n0.txt', 'testbed_kuramoto_k04.yaml'),
              ('2', 'REPLACE_kuramoto_n2.txt', 'testbed_kuramoto_k04_n2.yaml'),
              ('5', 'REPLACE_kuramoto_n5.txt', 'testbed_kuramoto_k04_n5.yaml'),
              ('10', 'REPLACE_kuramoto_n10.txt', 'testbed_kuramoto_k04_n10.yaml')],
        # The four properties the law actually promises: phase lock, the ring
        # radius, even angular spacing, and no collisions. order_R and d_min
        # are "higher is better"; the other two are errors.
        native=[('order_R', 1, '', 'Order parameter R (phase lock)', '{:.4f}', False, ''),
                ('radius_rms', 100, 'cm', 'Radial error (on the ring)', '{:.1f}', True, ''),
                ('angular_spacing_rms', 1, 'rad', 'Angular spacing error', '{:.3f}', True, ''),
                ('d_min', 100, 'cm', 'Smallest pair gap (safety)', '{:.0f}', False, '')],
        gain='velocity_gain',
        note='four agents on a 0.65 m ring, omega 0.35 rad/s',
    ),
    'distance': dict(
        title='distance formation',
        win=130.0,
        ramp=RAMP4,
        pattern='octahedron',
        runs=[('0', 'REPLACE_distance_n0.txt', 'testbed_hexagon_hybrid_k04.yaml'),
              ('2', 'REPLACE_distance_n2.txt', 'testbed_hexagon_hybrid_k04_n2.yaml'),
              ('5', 'REPLACE_distance_n5.txt', 'testbed_hexagon_hybrid_k04_n5.yaml'),
              ('10', 'REPLACE_distance_n10.txt', 'testbed_hexagon_hybrid_k04_n10.yaml')],
        # edge_rms is the promise -- the shape is defined by distances alone.
        # W_lyap is the convergence certificate the proof uses. Both are logged
        # natively here because the static config leaves breathing disabled, so
        # the recorder's reference guard never fires.
        native=[('edge_rms', 100, 'cm', 'Edge-length error (the promise)', '{:.2f}', True, ''),
                ('shape_err', 100, 'cm', 'Shape error after rigid fit', '{:.2f}', True, ''),
                ('W_lyap', 1, '', 'Lyapunov function W', '{:.4f}', True, ''),
                ('d_min', 100, 'cm', 'Smallest pair gap (safety)', '{:.0f}', False, '')],
        gain='gain_kv',
        note='six agents, octahedron, 0.7 m sides, drone1 anchored',
    ),
    'flocking': dict(
        title='flocking',
        win=124.0,
        ramp=RAMP4,
        pattern='flocking lattice',
        runs=[('0', 'flocking_20260929_143348.txt', 'testbed_flocking_hybrid_s2_n0.yaml'),
              ('2', 'flocking_20260929_144718.txt', 'testbed_flocking_hybrid_s2_n2.yaml'),
              ('5', 'flocking_20260929_145234.txt', 'testbed_flocking_hybrid_s2_n5.yaml'),
              ('10', 'flocking_20260929_150050.txt', 'testbed_flocking_hybrid_s2_n10.yaml')],
        # (record column, scale, unit, panel title, format, "higher is worse")
        # Every one of these is computed from /state, so the injected noise is
        # inside them too. For the three distance-based ones that only biases
        # them upwards, which makes their flatness conservative. vel_spread is
        # different: it reads the least-squares velocity, where the fit has
        # already multiplied the position noise by 11, so at 10 mm the noise
        # alone puts ~0.11 m/s into drone1's velocity -- more than the whole
        # rise the panel shows. It is marked as the artefact it is.
        native=[('lattice_err', 100, 'cm', 'Lattice error (cohesion)', '{:.1f}', True, ''),
                ('vel_spread', 100, 'cm/s', 'Velocity spread (matching)', '{:.1f}', True,
                 'artefact: this reads the 11x-amplified noisy velocity'),
                ('d_min', 100, 'cm', 'Smallest pair gap (safety)', '{:.0f}', False, ''),
                ('connected', 100, '% of ticks', 'Graph connected (connectivity)', '{:.1f}', False, '')],
        gain='gain_c2_gamma',
        note='target spacing 0.7 m, gamma circle r = 0.6 m at 0.075 rad/s',
    ),
    'coverage': dict(
        title='coverage',
        win=112.0,
        ramp=RAMP5,
        pattern='coverage configuration',
        runs=[('0', 'coverage_20260929_151743.txt', 'testbed_coverage_c2_n0.yaml'),
              ('2', 'coverage_20260929_152418.txt', 'testbed_coverage_c2_n2.yaml'),
              ('5', 'coverage_20260929_152810.txt', 'testbed_coverage_c2_n5.yaml'),
              ('10', 'coverage_20260929_153141.txt', 'testbed_coverage_c2_n10.yaml'),
              ('20', 'coverage_20260929_153520.txt', 'testbed_coverage_c2_n20.yaml')],
        # All four are computed from /state, so the injected noise is inside
        # them. It can only push a distance-to-centroid or a cost upwards, so
        # the flatness below is if anything an understatement.
        native=[('cdist_mean', 100, 'cm', 'Mean distance to own centroid', '{:.1f}', True, ''),
                ('cdist_max', 100, 'cm', 'Worst distance to own centroid', '{:.1f}', True, ''),
                ('H', 1, 'cost', 'Locational cost H', '{:.3f}', True, ''),
                ('h_rose', 100, '% of ticks', 'Ticks where H rose', '{:.0f}', True, '')],
        gain='gain_kd',
        note='region half-width 1.3 m, hotspot orbiting at 0.3 rad/s on r = 0.6 m',
    ),
}

COLOUR, X, RUNGS = {}, {}, []


def name(k):
    return 'no added noise' if k == '0' else f'{k} mm'


def read_real(path, ids):
    """The real drones named in the record's own header, else REAL."""
    try:
        with open(path) as fh:
            for line in fh:
                if not line.startswith('#'):
                    break
                m = re.search(r'real drones\s+(.*)', line)
                if m:
                    got = tuple(p.split('=')[0].strip() for p in m.group(1).split(','))
                    got = tuple(i for i in got if i in ids)
                    if got:
                        return got
    except OSError:
        pass
    return tuple(i for i in REAL if i in ids)


def read_k(path, fallback):
    """The own-velocity gain the flight was flown at, from the recorder's note.

    Flocking's k is state-dependent (c2_gamma + c2_alpha * sum of bump weights),
    so it cannot be read off the config the way coverage's gain_kd can; the
    launch writes the value it computed into the header.
    """
    try:
        with open(path) as fh:
            for line in fh:
                if not line.startswith('#'):
                    break
                m = re.search(r'\bk\s+([0-9.]+)\s+k\*tau', line)
                if m:
                    return float(m.group(1))
    except OSError:
        pass
    return fallback


def bp(x, lo, hi):
    return sosfiltfilt(butter(4, [lo, hi], 'band', fs=1 / DT, output='sos'), x)


def analyse(spec, label, rec, cfgname):
    cfg = yaml.safe_load(open(os.path.join(CFG, cfgname)))
    ids = [d['id'] for d in cfg['drones']]
    params = dict(cfg['algorithm']['params'] or {})
    clamp = float(params.get('max_accel', 3.5))
    noise = float(params.get('mocap_noise', 0.0))

    cols, D = A.load_record(rec)
    t = D[:, 0]
    P = np.stack([np.stack([D[:, cols.index(f'x_{i}')], D[:, cols.index(f'y_{i}')]], 1)
                  for i in ids], 1)
    last = A.live_end(P)
    _, _, _, _, t_start = A.classify(t, P, last, ids)
    reals = read_real(rec, ids)
    j = ids.index(reals[0])

    t0 = t_start + WIN_SKIP
    tg = np.arange(t0, t0 + spec['win'], DT)
    if tg[-1] > t[last]:
        raise SystemExit(f'{label} mm: record ends at {t[last]:.0f} s, before the '
                         f"{spec['win']:.0f} s window closes at {tg[-1]:.0f} s")
    Pg = np.stack([np.stack([np.interp(tg, t, P[:, n, ax]) for ax in (0, 1)], 1)
                   for n in range(len(ids))], 1)
    tilt = np.interp(tg, t, D[:, cols.index(f'tilt_{reals[0]}')])
    z = np.interp(tg, t, D[:, cols.index(f'z_{reals[0]}')])

    # The designed pattern: this config in the noiseless simulator, started from
    # where the fleet actually was when the algorithm engaged. Started from the
    # config marks instead it would carry the hover offset as a fixed error in
    # every rung, which is not what this lever is about.
    ks = int(np.searchsorted(t, t_start - 0.5))
    cfg0 = copy.deepcopy(cfg)
    for n, d in enumerate(cfg0['drones']):
        d['initial_position'] = P[ks, n].tolist()
        d['initial_velocity'] = [0.0, 0.0]
    _, ts0, _, ph0, _ = S.run(cfg0, tg[-1] - t_start + 1.0)
    Ps0 = np.array(ph0)
    Dg = np.stack([np.stack([np.interp(tg - t_start, ts0, Ps0[:, n, ax])
                             for ax in (0, 1)], 1) for n in range(len(ids))], 1)

    dev = np.hypot(*(Pg[:, j] - Dg[:, j]).T)
    vj = [n for n, i in enumerate(ids) if i not in reals]
    dev_virt = (np.mean([np.hypot(*(Pg[:, n] - Dg[:, n]).T) for n in vj], axis=0)
                if vj else np.zeros(len(tg)))

    zc = Pg[:, j, 0] + 1j * Pg[:, j, 1]
    ripple = bp(zc.real, *SHAKE_BAND) + 1j * bp(zc.imag, *SHAKE_BAND)
    V = np.gradient(Pg, DT, axis=0)
    Acc = np.gradient(V, DT, axis=0)
    speed = np.hypot(*V[:, j].T)
    accel = np.hypot(*Acc[:, j].T)

    # The commanded acceleration, recomputed by running the same algorithm
    # object over the recorded positions. The aircraft's own controller saw
    # NOISY, DELAYED states, so this is the command the pattern asks for, not
    # the one the drone was given: the clip fraction below is a lower bound.
    obj = get_algorithm(cfg['algorithm']['name'])
    obj.configure(dict(params), list(ids))
    u = np.empty((len(tg), 2))
    for r in range(len(tg)):
        ds = {i: DroneState(i, Pg[r, n].copy(), V[r, n].copy())
              for n, i in enumerate(ids)}
        u[r] = obj.compute_controls(ds, DT)[reals[0]].acceleration[:2]

    nat = {}
    for key, *_ in spec['native']:
        if key == 'h_rose':
            H = np.interp(tg, t, D[:, cols.index('H')])
            nat[key] = float(np.mean(np.diff(H) > 1.64e-5))
            continue
        v = np.interp(tg, t, D[:, cols.index(key)])
        nat[key] = float(np.nanmean(v))
    nat_series = {key: np.interp(tg, t, D[:, cols.index(key)])
                  for key, *_ in spec['native'] if key != 'h_rose'}

    n3 = len(dev) // 3
    thirds = [float(np.sqrt(np.mean(dev[i * n3:(i + 1) * n3] ** 2))) for i in range(3)]
    return dict(
        label=label, noise=noise, rec=rec, cfg_name=cfgname, clamp=clamp,
        k=read_k(rec, float(params.get(spec['gain'], 0.0))),
        reals=reals, n_real=len(reals), j=j, ids=ids,
        t_start=t_start, live=t[last], tau=tg - t_start, Pg=Pg, Dg=Dg,
        dev=dev, dev_vec=Pg[:, j] - Dg[:, j], dev_virt=dev_virt, thirds=thirds,
        dev_rms=float(np.sqrt(np.mean(dev ** 2))),
        dev_med=float(np.median(dev)), dev_p95=float(np.percentile(dev, 95)),
        dev_max=float(dev.max()),
        virt_rms=float(np.sqrt(np.mean(dev_virt ** 2))),
        ripple=float(np.sqrt(np.mean(np.abs(ripple) ** 2))),
        speed=float(np.median(speed)), speed_p95=float(np.percentile(speed, 95)),
        accel_p95=float(np.percentile(accel, 95)), accel_max=float(accel.max()),
        tilt_med=float(np.median(tilt)), tilt_p95=float(np.percentile(tilt, 95)),
        tilt_max=float(tilt.max()),
        z_sd=float(np.std(z)),
        u_med=float(np.median(np.hypot(*u.T))),
        clip=float(np.mean(np.any(np.abs(u) > clamp * 0.999, axis=1))),
        radius_max=float(np.hypot(*Pg[:, j].T).max()),
        nat=nat, nat_series=nat_series,
    )


def add_nulls(runs, draws=200, seed=0):
    """What each rung would read if only the SENSOR had got noisier.

    The control run's own trajectory and its own error against the design, put
    through each rung's noise and re-measured. A bar that sits at its null is
    measuring the injection, not the aircraft; the gap above the null is the
    part the aircraft actually did. Only the two position-derived measures need
    this -- tilt comes from /poses and is never noisy.
    """
    rng = np.random.default_rng(seed)
    base, P0 = runs[0], runs[0]['Pg'][:, runs[0]['j']]
    for r in runs:
        if r['noise'] <= 0:
            r['dev_rms_null'] = base['dev_rms']
            r['ripple_null'] = base['ripple']
            continue
        dv, rp = [], []
        for _ in range(draws):
            e = rng.normal(0.0, r['noise'], P0.shape)
            dv.append(np.sqrt(np.mean(np.hypot(*(base['dev_vec'] + e).T) ** 2)))
            z = (bp(P0[:, 0] + e[:, 0], *SHAKE_BAND)
                 + 1j * bp(P0[:, 1] + e[:, 1], *SHAKE_BAND))
            rp.append(np.sqrt(np.mean(np.abs(z) ** 2)))
        r['dev_rms_null'] = float(np.mean(dv))
        r['ripple_null'] = float(np.mean(rp))


def table(spec, runs):
    k = runs[0]['k']
    print(f"\n{spec['title']} measurement-noise ladder, own-velocity gain k = {k:.2f} "
          f"(k*tau {k * TAU:.2f}), drone1 real, {spec['win']:.0f} s window from "
          f"engage + {WIN_SKIP:.0f} s")
    print(f"  {spec['note']}\n")
    print('       deviation from the designed pattern (cm)        sim    ripple (cm)   '
          'tilt, clean channel (deg)')
    print('noise  RMS  (null)  median   p95    max  1st/2nd/3rd  agents   meas (null)   '
          'median  p95   max')
    for r in runs:
        th = '/'.join(f'{v * 100:.1f}' for v in r['thirds'])
        print(f"{r['label']:>4}mm {r['dev_rms'] * 100:5.1f} ({r['dev_rms_null'] * 100:4.1f}) "
              f"{r['dev_med'] * 100:6.1f} {r['dev_p95'] * 100:6.1f} {r['dev_max'] * 100:6.1f}  "
              f"{th:>14}   {r['virt_rms'] * 100:5.1f}   "
              f"{r['ripple'] * 100:4.2f} ({r['ripple_null'] * 100:4.2f})   "
              f"{r['tilt_med']:5.1f} {r['tilt_p95']:5.1f} {r['tilt_max']:5.1f}")
    print('  (null) = what the CONTROL aircraft would read through that rung\'s noisier')
    print('  sensor, 200 draws. The recorder takes x and y from /state, so anything')
    print('  derived from position carries the injection; tilt comes from /poses.')

    base, top = runs[0], runs[-1]
    print(f"\ndeviation grew {top['dev_rms'] / base['dev_rms']:.1f}x from the control to "
          f"{top['label']} mm ({base['dev_rms'] * 100:.1f} -> {top['dev_rms'] * 100:.1f} cm RMS), "
          f"and sits {top['dev_rms'] / top['dev_rms_null']:.2f}x its own sensor-only null; "
          f"ripple {top['ripple'] / base['ripple']:.1f}x and "
          f"{top['ripple'] / top['ripple_null']:.2f}x its null; "
          f"median tilt {top['tilt_med'] / base['tilt_med']:.1f}x on the clean channel")
    print(f"flown speed and acceleration are NOT reported: differencing the recorded "
          f"position\nat 10 Hz multiplies the injected noise by 14 and 245, which is most "
          f"of what\nthose two measures were showing.")

    print(f"\nthe algorithm's own success metrics over the same window:")
    hdr = ''.join(f'{key:>18}' for key, *_ in spec['native'])
    print(f"{'noise':>7}{hdr}")
    for r in runs:
        cells = ''
        for key, sc, unit, _t, fmt, _w, _x in spec['native']:
            cells += f'{fmt.format(r["nat"][key] * sc):>18}'
        print(f"{r['label'] + 'mm':>7}{cells}")
    for key, sc, unit, ttl, fmt, worse, extra in spec['native']:
        vals = np.array([r['nat'][key] for r in runs])
        span = (vals.max() - vals.min()) / abs(np.mean(vals)) * 100
        direction = 'worse' if (vals[-1] > vals[0]) == worse else 'better'
        print(f"  {key:<12} spread {span:4.1f}% of its own mean across the whole ladder, "
              f"{top['label']} mm reads {direction} than the control"
              + (f"  [{extra}]" if extra else ''))

    print(f"\nthe noise the brake sees: the velocity fit over 10 samples multiplies each "
          f"axis's\nposition noise by 11, and the -k*v term multiplies that by k = {k:.2f}, so")
    for r in runs:
        if r['noise'] > 0:
            print(f"  {r['label']:>4}mm  {11 * r['noise'] * r['k']:.2f} m/s2 of pure noise "
                  f"in the commanded acceleration, against a pattern that asks for "
                  f"{r['u_med']:.2f}")
    print(f"\nevery rung stayed inside the limits: max radius " +
          ', '.join(f"{r['radius_max']:.2f}" for r in runs) +
          f" m (geofence 1.5), and the pattern itself never reached the "
          f"{runs[0]['clamp']:.1f} m/s2 clamp")
    if spec['title'] == 'flocking':
        print("\nthe 2 mm record has a 0.9 s tracking loss at t = 144.0-144.8 s (drone1 jumps\n"
              "0.7 m and drops to z = 0.83 m on a tiring battery). It is outside this window\n"
              f"by design; over the trochoidal ladder's 148 s window that one event alone\n"
              "takes the 2 mm rung from 4.8 to 21.0 cm RMS and inverts the ladder.")


def _noise_axis(ax, short=False):
    xs = [X[k] for k in RUNGS]
    ax.set_xticks(xs)
    ax.set_xticklabels([k if short else name(k) for k in RUNGS], fontsize=8.5)
    ax.set_xlim(min(xs) - 0.7, max(xs) + 0.7)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def fig_deviation(spec, runs, out):
    """The headline: how far drone1 flew from the designed pattern."""
    fig, axs = plt.subplots(1, 2, figsize=(12.2, 4.8),
                            gridspec_kw=dict(width_ratios=[2.3, 1]))
    ax = axs[0]
    for r in runs:
        ax.plot(r['tau'], r['dev'] * 100, color=COLOUR[r['label']], lw=1.2,
                label=name(r['label']))
    ax.set_xlim(runs[0]['tau'][0], runs[0]['tau'][-1])
    ax.set_ylim(0, max(r['dev_max'] for r in runs) * 100 * 1.22)
    ax.set_xlabel('seconds since the algorithm started')
    ax.set_ylabel('distance from the designed position (cm)')
    ax.set_title('Deviation from the designed pattern over the flight',
                 loc='left', fontsize=10)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(fontsize=8.5, ncol=len(runs), loc='upper left', framealpha=0.9,
              title="noise on drone1's VICON position", title_fontsize=8.5)

    ax = axs[1]
    bx = ax.boxplot([r['dev'] * 100 for r in runs],
                    positions=[X[r['label']] for r in runs], widths=0.6,
                    whis=(5, 95), patch_artist=True, showfliers=True,
                    flierprops=dict(marker='.', ms=2, mfc=MUTED, mec='none', alpha=0.25),
                    medianprops=dict(color=INK, lw=1.6),
                    whiskerprops=dict(color=MUTED, lw=1.0),
                    capprops=dict(color=MUTED, lw=1.0))
    for patch, r in zip(bx['boxes'], runs):
        patch.set_facecolor(COLOUR[r['label']])
        patch.set_edgecolor('white')
        patch.set_linewidth(2)
    for r in runs:
        ax.annotate(f"{r['dev_med'] * 100:.1f}", (X[r['label']], r['dev_med'] * 100),
                    textcoords='offset points', xytext=(13, -3), fontsize=8.5,
                    color=INK, ha='left')
    _noise_axis(ax)
    ax.set_xlabel("noise added to drone1's VICON position")
    ax.set_ylabel('distance from the designed position (cm)')
    ax.set_title('Every sample of the flight', loc='left', fontsize=10)
    ax.set_xticklabels([f'{k} mm' if k != '0' else '0\n(control)' for k in RUNGS],
                       fontsize=8.5)

    k = runs[0]['k']
    fig.suptitle(f"Adding noise to drone1's position sensor moves it off the designed "
                 f"{spec['pattern']}\n"
                 'box = middle half of the flight, whiskers 5-95%, label = median; '
                 f"{spec['win']:.0f} s window, identical gains (k = {k:.2f}, "
                 f"k*tau = {k * TAU:.2f}) in all {len(runs)} flights",
                 x=0.008, y=0.985, va='top', ha='left', fontsize=11.5,
                 fontweight='bold', color=INK)
    fig.tight_layout()
    fig.subplots_adjust(top=0.85)
    fig.savefig(os.path.join(out, '1_deviation.png'), dpi=150)
    plt.close(fig)


def fig_paths(spec, runs, out):
    """What the deviation looks like as a flown path."""
    fig, axs = plt.subplots(1, len(runs), figsize=(2.85 * len(runs), 4.0))
    axs = np.atleast_1d(axs)
    lim = max(max(np.abs(r['Pg'][:, r['j']]).max(), np.abs(r['Dg'][:, r['j']]).max())
              for r in runs) * 1.12
    for ax, r in zip(axs, runs):
        j = r['j']
        ax.plot(r['Dg'][:, j, 0], r['Dg'][:, j, 1], color=DESIGN, lw=3.0,
                solid_capstyle='round')
        ax.plot(r['Pg'][:, j, 0], r['Pg'][:, j, 1], color=COLOUR[r['label']],
                lw=0.9, alpha=0.9)
        ax.set_aspect('equal')
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_xlabel('x, m')
        ax.grid(color=GRID, lw=0.7)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        ax.set_title(f"{name(r['label'])}\nRMS deviation {r['dev_rms'] * 100:.1f} cm",
                     loc='left', fontsize=9.5, color=INK)
    axs[0].set_ylabel('y, m')
    h = [Line2D([], [], color=DESIGN, lw=3.0),
         Line2D([], [], color=COLOUR[RUNGS[len(RUNGS) // 2]], lw=1.2)]
    fig.legend(h, ['designed pattern (noiseless simulation of the same config)',
                   'where drone1 actually flew (darker = more noise)'],
               loc='lower center', ncol=2, fontsize=9, frameon=False,
               bbox_to_anchor=(0.5, 0.0))
    fig.suptitle(f"drone1's flown path against the designed one, as the position "
                 f"noise rises",
                 x=0.008, y=0.985, va='top', ha='left', fontsize=11.5,
                 fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    fig.subplots_adjust(top=0.85)
    fig.savefig(os.path.join(out, '2_paths.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def fig_two_benchmarks(spec, runs, out):
    """The point of these ladders: the flight degrades, the published metrics do not.

    Top row is how the aircraft actually flew, bottom row is what the source
    paper would plot. Two honesty devices, both needed because the recorder's
    x and y ARE the noisy reading:

      - the two position-derived panels carry a tick at the sensor-only null,
        so the reader can see how much of the bar the injection alone explains;
      - bottom panels whose whole ladder spans less than 10% of their own mean
        are drawn in a flat neutral, because that is the finding.

    Tilt is on the third and fourth panels rather than a differentiated speed
    or acceleration: it comes from /poses, so it is the one motion measure the
    injection cannot reach.
    """
    flight = [
        ('dev_rms', 100, 'cm', 'Deviation from the design', '{:.1f}', 'dev_rms_null'),
        ('ripple', 100, 'cm', f'Oscillation, {SHAKE_BAND[0]}-{SHAKE_BAND[1]} Hz',
         '{:.2f}', 'ripple_null'),
        ('tilt_med', 1, 'deg', 'Tilt, median (clean channel)', '{:.1f}', None),
        ('tilt_p95', 1, 'deg', 'Tilt, 95th percentile (clean channel)', '{:.1f}', None),
    ]
    # Laid out by hand rather than with tight_layout: a left-aligned title that
    # overruns its panel counts towards that column's tight bounding box, so
    # tight_layout answers long titles by shrinking the axes to a sliver.
    fig, axs = plt.subplots(2, 4, figsize=(13.4, 7.4))
    fig.subplots_adjust(left=0.055, right=0.99, top=0.775, bottom=0.10,
                        wspace=0.30, hspace=0.78)

    def panel(ax, vals, fmt, unit, title, colours, note, note_ink, nulls=None):
        ax.bar([X[r['label']] for r in runs], vals, width=0.62, color=colours)
        top = max(vals)
        if nulls is not None:
            for r, nv in zip(runs, nulls):
                ax.plot([X[r['label']] - 0.36, X[r['label']] + 0.36], [nv, nv],
                        color=INK, lw=1.4, solid_capstyle='butt', zorder=4)
            top = max(top, max(nulls))
        for r, v in zip(runs, vals):
            ax.annotate(fmt.format(v), (X[r['label']], v), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=8, color=INK)
        _noise_axis(ax, short=True)
        ax.set_ylim(0, top * 1.26)
        ax.set_ylabel(unit)
        # pad lifts the title clear so the note can sit on its own line below
        ax.set_title(title, loc='left', fontsize=9.5, color=INK, pad=17)
        ax.annotate(note, (0.0, 1.012), xycoords='axes fraction', ha='left',
                    va='bottom', fontsize=8.5, color=note_ink)

    for ax, (key, sc, unit, title, fmt, nullkey) in zip(axs[0], flight):
        vals = [r[key] * sc for r in runs]
        nulls = [r[nullkey] * sc for r in runs] if nullkey else None
        # Control to top rung, not max over min: on the panels where the
        # control happens to be the highest bar, a max/min ratio would read
        # as growth when the ladder actually goes the other way.
        grew = vals[-1] / vals[0]
        if nulls is None:
            note = f'x{grew:.1f} from the control to {RUNGS[-1]} mm'
            ink = INK if grew > 1.15 else MUTED
        else:
            # Judge the top rung against its own null too: the question is how
            # much of the bar the aircraft is responsible for.
            exc = vals[-1] / nulls[-1]
            note = (f'x{grew:.1f} from the control, '
                    f'x{exc:.1f} its sensor-only null')
            ink = INK if exc > 1.3 else MUTED
        panel(ax, vals, fmt, unit, title, [COLOUR[r['label']] for r in runs],
              note, ink, nulls)

    n_flat = 0
    for ax, (key, sc, unit, title, fmt, worse, extra) in zip(axs[1], spec['native']):
        vals = [r['nat'][key] * sc for r in runs]
        span = (max(vals) - min(vals)) / abs(np.mean(vals)) * 100
        flat = span < 10.0
        # A moving bottom-row metric is only bad news if it moved the wrong way
        # AND the movement is the aircraft rather than the measurement.
        way = '' if flat else (', worse' if (vals[-1] > vals[0]) == worse
                               else ', but better')
        note = extra or f'spans {span:.0f}% of its own mean{way}'
        n_flat += bool(flat or way.endswith('better') or extra)
        panel(ax, vals, fmt, unit, title,
              FLAT if (flat or extra) else [COLOUR[r['label']] for r in runs],
              note, MUTED if (flat or extra) else INK)

    # Row banners, spaced out by hand: matplotlib has no letter-spacing, and
    # these have to read as headings rather than as another panel title.
    for ax, txt in ((axs[0][0], 'H O W   T H E   A I R C R A F T   F L E W'),
                    (axs[1][0], "W H A T   T H E   A L G O R I T H M ' S   O W N   "
                                "M E T R I C S   R E P O R T E D")):
        ax.text(-0.14, 1.32, txt, transform=ax.transAxes, fontsize=9,
                fontweight='bold', color=INK)

    fig.supxlabel("noise added to drone1's VICON position, mm per axis",
                  x=0.008, ha='left', fontsize=9.5, color=MUTED)
    n_nat = len(spec['native'])
    fig.suptitle(f"The noise degrades the {spec['title']} flight, but "
                 f"{n_flat} of the {n_nat} metrics the theory is judged on never "
                 f"report it",
                 x=0.008, y=0.988, va='top', ha='left', fontsize=12,
                 fontweight='bold', color=INK)
    # Kept out of the suptitle so it can be set smaller and lighter: three
    # bold lines at title size overrun the figure width.
    fig.text(0.008, 0.945,
             "Black ticks are the sensor-only null: what the CONTROL aircraft "
             "would read through that rung's noisier sensor. The recorder takes "
             "x and y from /state, so\nanything derived from position carries the "
             "injection; tilt comes from /poses and never does. Grey bars span "
             "under 10% of their own mean, or move only\nbecause the measurement "
             "moved. Gains, clamp and flight time are identical in every flight.",
             va='top', ha='left', fontsize=9, color=MUTED, linespacing=1.5)
    fig.savefig(os.path.join(out, '3_two_benchmarks.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--algo', choices=sorted(SPEC), default='flocking')
    ap.add_argument('--out', default=None)
    ap.add_argument('--runs', help='override the records: "0=rec.txt,2=rec.txt,..." '
                                   '-- configs are taken from the spec')
    a = ap.parse_args()
    spec = SPEC[a.algo]
    if a.runs:
        by_label = {lab: cfg for lab, _, cfg in spec['runs']}
        spec = dict(spec, runs=[(lab, rec, by_label[lab]) for lab, rec in
                                (c.split('=', 1) for c in a.runs.split(','))])
    out = a.out or os.path.join(ROOT, 'docs', 'figures', f'noise_ladder_{a.algo}')
    os.makedirs(out, exist_ok=True)

    runs = []
    for label, rec, cfgname in spec['runs']:
        path = os.path.join(LOGS, rec)
        if not os.path.isfile(path):
            print(f'  {label} mm: no record -- left out')
            continue
        runs.append(analyse(spec, label, path, cfgname))
    if not runs:
        raise SystemExit('no records found')
    RUNGS[:] = [r['label'] for r in runs]
    COLOUR.clear()
    COLOUR.update(dict(zip(RUNGS, spec['ramp'][:len(RUNGS)])))
    X.clear()
    X.update({k: n for n, k in enumerate(RUNGS)})

    add_nulls(runs)
    table(spec, runs)
    fig_deviation(spec, runs, out)
    fig_paths(spec, runs, out)
    fig_two_benchmarks(spec, runs, out)

    summary = {r['label']: {k: r[k] for k in
                            ('noise', 'k', 'dev_rms', 'dev_med', 'dev_p95', 'dev_max',
                             'virt_rms', 'ripple', 'dev_rms_null', 'ripple_null',
                             'speed', 'speed_p95', 'accel_p95', 'accel_max',
                             'tilt_med', 'tilt_p95', 'tilt_max', 'clip', 'u_med',
                             'radius_max', 'thirds', 'n_real', 'z_sd')}
               for r in runs}
    for r in runs:
        s = summary[r['label']]
        s['rec'] = os.path.basename(r['rec'])
        s['cfg'] = r['cfg_name']
        s['reals'] = list(r['reals'])
        s['native'] = dict(r['nat'])
        s['window_s'] = spec['win']
    with open(os.path.join(out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'\nfigures in {out}')


if __name__ == '__main__':
    main()
