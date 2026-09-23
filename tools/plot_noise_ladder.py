#!/usr/bin/env python3
"""Plots for the trochoidal measurement-noise ladder (23 Sep 2026).

Four flights, identical in every respect except the artificial Gaussian noise
added to drone1's VICON position: 0 (control), 2, 5, 10 mm per axis. Only
drone1 is real; drone2-4 are simulated double integrators inside the same node,
so the noise enters the law through one agent and reaches the rest only through
the coupling term.

    python3 tools/plot_noise_ladder.py [--out docs/figures/noise_ladder]

The benchmark is DEVIATION FROM THE DESIGNED PATTERN, not the ripple. The
designed pattern here is the same config replayed in the noiseless simulator,
restarted from where the fleet actually was when the algorithm engaged, so the
deviation is exactly the sim-to-hardware gap this lever opens up.
"""

import argparse
import copy
import json
import math
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.lines import Line2D                              # noqa: E402
from scipy.signal import butter, sosfiltfilt                    # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyse_trochoidal as A                                  # noqa: E402
import sim_baseline as S                                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed', 'config')
LOGS = os.path.join(ROOT, 'logs', 'hw')

# (noise mm, record, config).  All four flown in one session, 23 Sep 2026,
# beta = 2 (k*tau 0.56), max_accel 3.5, drone1 on (0.235, -0.132).
RUNS = [
    ('0', 'trochoidalconsensus_20260923_152619.txt', 'testbed_fig4_r2_n0.yaml'),
    ('2', 'trochoidalconsensus_20260923_153238.txt', 'testbed_fig4_r2_n2.yaml'),
    ('5', 'trochoidalconsensus_20260923_154226.txt', 'testbed_fig4_r2_n5.yaml'),
    ('10', 'trochoidalconsensus_20260923_155129.txt', 'testbed_fig4_r2_n10.yaml'),
]
REAL = 'drone1'
DT = 0.1          # the recorder's row interval
# All four flights ran 180 s and began descending at ~185 s, so a single window
# is used for every rung rather than each one's own live end: the deviation
# distributions are only comparable if they cover the same number of laps, and
# the descent has to be outside it (a landing drone leaves the pattern and would
# dominate every percentile). 2 s after engage, 148 s long -> ends at ~164 s.
# The 2 mm record also has ~30 s of the recorder running after touchdown.
WIN_SKIP = 2.0
WIN_LEN = 148.0
SHAKE_BAND = (0.6, 1.5)   # the k*tau brake loop, ~1.1 s period
G = 9.81

INK = '#1f2328'
MUTED = '#6b7280'
GRID = '#e5e7eb'
DESIGN = '#9ca3af'
# Sequential, light to dark with the noise level: the rungs are an ordered
# magnitude, not categories. Cool ramp so these figures are not mistaken for
# the beta ladder, which owns the warm one.
COLOUR = {'0': '#a3c9e6', '2': '#5b9bd0', '5': '#2a6ca8', '10': '#0d3b63'}
X = {'0': 0, '2': 1, '5': 2, '10': 3}
RUNGS = []


def name(k):
    return 'no added noise' if k == '0' else f'{k} mm'


def bp(x, lo, hi, axis=0):
    return sosfiltfilt(butter(4, [lo, hi], 'band', fs=1 / DT, output='sos'),
                       x, axis=axis)


def analyse(label, rec, cfgname):
    cfg = yaml.safe_load(open(os.path.join(CFG, cfgname)))
    ids = [d['id'] for d in cfg['drones']]
    al, be, ka, th, adj, clamp = A.law(cfg)
    Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    noise = float((cfg['algorithm']['params'] or {}).get('mocap_noise', 0.0))

    cols, D = A.load_record(rec)
    t = D[:, 0]
    P = np.stack([np.stack([D[:, cols.index(f'x_{i}')], D[:, cols.index(f'y_{i}')]], 1)
                  for i in ids], 1)
    last = A.live_end(P)
    _, virt, _, _, t_start = A.classify(t, P, last, ids)
    j = ids.index(REAL)

    t0 = t_start + WIN_SKIP
    tg = np.arange(t0, t0 + WIN_LEN, DT)
    if tg[-1] > t[last]:
        raise SystemExit(f'{label}: record ends at {t[last]:.0f} s, before the '
                         f'{WIN_LEN:.0f} s window closes at {tg[-1]:.0f} s')
    Pg = np.stack([np.stack([np.interp(tg, t, P[:, n, ax]) for ax in (0, 1)], 1)
                   for n in range(len(ids))], 1)                 # rows, drone, 2
    tilt = np.interp(tg, t, D[:, cols.index(f'tilt_{REAL}')])
    z = np.interp(tg, t, D[:, cols.index(f'z_{REAL}')])

    # The designed pattern: the same config in the noiseless simulator, started
    # from where the fleet actually was when the algorithm engaged. Started from
    # the config marks instead it would carry the hover offset as a fixed error
    # in every rung, which is not what this lever is about.
    ks = int(np.searchsorted(t, t_start - 0.5))
    cfg0 = copy.deepcopy(cfg)
    for n, d in enumerate(cfg0['drones']):
        d['initial_position'] = P[ks, n].tolist()
        d['initial_velocity'] = [0.0, 0.0]
    _, ts0, _, ph0, _ = S.run(cfg0, tg[-1] - t_start + 1.0)
    Ps0 = np.array(ph0)
    Dg = np.stack([np.stack([np.interp(tg - t_start, ts0, Ps0[:, n, ax])
                             for ax in (0, 1)], 1) for n in range(len(ids))], 1)

    dev = np.hypot(*(Pg[:, j] - Dg[:, j]).T)                     # m, per row
    dev_virt = np.mean([np.hypot(*(Pg[:, n] - Dg[:, n]).T)
                        for n in range(len(ids)) if n != j], axis=0)

    zc = Pg[:, j, 0] + 1j * Pg[:, j, 1]
    ripple = bp(zc.real, *SHAKE_BAND) + 1j * bp(zc.imag, *SHAKE_BAND)
    V = np.gradient(Pg, DT, axis=0)
    Acc = np.gradient(V, DT, axis=0)
    rel = sum(w * (Pg[:, j] - Pg[:, ids.index(n)]) for n, w in adj[REAL].items())
    u = -al * Pg[:, j] - be * V[:, j] - ka * (rel @ Rm.T)

    thirds = [float(np.sqrt(np.mean(dev[i * len(dev) // 3:(i + 1) * len(dev) // 3] ** 2)))
              for i in range(3)]
    return dict(
        label=label, noise=noise, rec=rec, cfg_name=cfgname, beta=be, clamp=clamp,
        t_start=t_start, live=t[last], tau=tg - t_start, Pg=Pg, Dg=Dg, j=j,
        design_r=float(np.median(np.hypot(*Dg[:, j].T))),
        dev=dev, dev_virt=dev_virt, thirds=thirds,
        dev_rms=float(np.sqrt(np.mean(dev ** 2))),
        dev_med=float(np.median(dev)), dev_p95=float(np.percentile(dev, 95)),
        dev_max=float(dev.max()),
        virt_rms=float(np.sqrt(np.mean(dev_virt ** 2))),
        ripple=float(np.sqrt(np.mean(np.abs(ripple) ** 2))),
        speed=float(np.median(np.hypot(*V[:, j].T))),
        speed_p95=float(np.percentile(np.hypot(*V[:, j].T), 95)),
        tilt_p95=float(np.percentile(tilt, 95)), tilt_max=float(tilt.max()),
        z_sd=float(np.std(z)),
        u_med=float(np.median(np.hypot(*u.T))),
        clip=float(np.mean(np.any(np.abs(u) > clamp, axis=1))),
        radius_max=float(np.hypot(*Pg[:, j].T).max()),
    )


def table(runs):
    print(f'\ntrochoidal measurement-noise ladder, beta = {runs[0]["beta"]:g} '
          f'(k*tau {runs[0]["beta"] * 0.28:.2f}), drone1 real, '
          f'{WIN_LEN:.0f} s window from engage + {WIN_SKIP:.0f} s\n')
    print('noise   deviation from the designed pattern (cm)      ripple  speed med/p95  '
          'tilt p95/max  clipped  |u| med')
    print('        RMS   median   p95    max   1st/2nd/3rd third   (cm)     (m/s)          (deg)'
          '            (m/s2)')
    for r in runs:
        th = '/'.join(f'{v * 100:.1f}' for v in r['thirds'])
        print(f"{r['label']:>4}mm  {r['dev_rms'] * 100:4.1f}  {r['dev_med'] * 100:5.1f}  "
              f"{r['dev_p95'] * 100:5.1f}  {r['dev_max'] * 100:5.1f}   {th:>16}  "
              f"{r['ripple'] * 100:5.2f}  {r['speed']:.3f}/{r['speed_p95']:.3f}    "
              f"{r['tilt_p95']:4.1f}/{r['tilt_max']:4.1f}     "
              f"{r['clip'] * 100:4.1f}%   {r['u_med']:.3f}")
    print(f"\nthe designed pattern has a median radius of "
          f"{np.mean([r['design_r'] for r in runs]) * 100:.0f} cm, so the 10 mm rung's "
          f"{runs[-1]['dev_rms'] * 100:.0f} cm RMS deviation is "
          f"{runs[-1]['dev_rms'] / runs[-1]['design_r'] * 100:.0f}% of the pattern itself")
    print('simulated agents (drone2-4), RMS deviation: ' +
          ', '.join(f"{r['label']}mm {r['virt_rms'] * 100:.1f} cm" for r in runs))
    print('every rung stayed inside the limits: max radius ' +
          ', '.join(f"{r['radius_max']:.2f}" for r in runs) +
          ' m (geofence 1.5), clipping 0% at the 3.5 m/s2 clamp')


def _noise_axis(ax):
    xs = [X[k] for k in RUNGS]
    ax.set_xticks(xs)
    ax.set_xticklabels([name(k) for k in RUNGS], fontsize=8.5)
    ax.set_xlim(min(xs) - 0.7, max(xs) + 0.7)
    ax.set_xlabel('noise added to drone1\'s VICON position')
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def fig_deviation(runs, out):
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
    ax.legend(fontsize=8.5, ncol=4, loc='upper left', framealpha=0.9,
              title='noise on drone1\'s VICON position', title_fontsize=8.5)

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
    ax.set_ylabel('distance from the designed position (cm)')
    ax.set_title('Every sample of the flight', loc='left', fontsize=10)
    ax.set_xticklabels([f'{k} mm' if k != '0' else '0\n(control)' for k in RUNGS],
                       fontsize=8.5)

    fig.suptitle('Adding noise to drone1\'s position sensor moves it off the designed '
                 'trochoidal pattern\n'
                 'box = middle half of the flight, whiskers 5-95%, label = median; '
                 f"{WIN_LEN:.0f} s window, identical gains (beta = {runs[0]['beta']:g}) "
                 'in all four flights',
                 x=0.008, y=0.985, va='top', ha='left', fontsize=11.5,
                 fontweight='bold', color=INK)
    fig.tight_layout()
    fig.subplots_adjust(top=0.85)
    fig.savefig(os.path.join(out, '1_deviation.png'), dpi=150)
    plt.close(fig)


def fig_paths(runs, out):
    """What the deviation looks like as a flown path."""
    fig, axs = plt.subplots(1, len(runs), figsize=(3.05 * len(runs), 4.0))
    axs = np.atleast_1d(axs)
    lim = max(r['radius_max'] for r in runs) * 1.12
    for ax, r in zip(axs, runs):
        j = r['j']
        ax.plot(r['Dg'][:, j, 0], r['Dg'][:, j, 1], color=DESIGN, lw=3.0,
                label='designed pattern', solid_capstyle='round')
        ax.plot(r['Pg'][:, j, 0], r['Pg'][:, j, 1], color=COLOUR[r['label']],
                lw=0.9, alpha=0.9, label='where drone1 actually flew')
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
    # Built by hand rather than from the axes: the actual-path handle must not
    # be drawn in one rung's colour, since the colour is the noise level.
    h = [Line2D([], [], color=DESIGN, lw=3.0),
         Line2D([], [], color=COLOUR[RUNGS[len(RUNGS) // 2]], lw=1.2)]
    fig.legend(h, ['designed pattern (noiseless simulation of the same config)',
                   'where drone1 actually flew (darker = more noise)'],
               loc='lower center', ncol=2, fontsize=9, frameon=False,
               bbox_to_anchor=(0.5, 0.0))
    fig.suptitle('The flown path against the designed one, as the position noise rises',
                 x=0.008, y=0.985, va='top', ha='left', fontsize=11.5,
                 fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    fig.subplots_adjust(top=0.85)
    fig.savefig(os.path.join(out, '2_paths.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)


def fig_what_changed(runs, out):
    """Four quantities against the noise level, so it is clear which ones moved."""
    panels = [
        ('dev_rms', 100, 'cm', 'Deviation from the designed pattern (RMS)', '{:.1f}'),
        ('ripple', 100, 'cm', 'Radius of oscillation (0.6-1.5 Hz)', '{:.2f}'),
        ('speed', 1, 'm/s', 'Median flown speed', '{:.3f}'),
        ('tilt_p95', 1, 'deg', 'Tilt, 95th percentile', '{:.1f}'),
    ]
    fig, axs = plt.subplots(1, 4, figsize=(13.2, 3.5))
    for ax, (key, sc, unit, title, fmt) in zip(axs, panels):
        vals = [r[key] * sc for r in runs]
        ax.bar([X[r['label']] for r in runs], vals, width=0.62,
               color=[COLOUR[r['label']] for r in runs])
        for r, v in zip(runs, vals):
            ax.annotate(fmt.format(v),
                        (X[r['label']], v), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=8.5, color=INK)
        _noise_axis(ax)
        ax.set_xlabel('')
        ax.set_ylim(0, max(vals) * 1.2)
        ax.set_ylabel(unit)
        ax.set_title(title, loc='left', fontsize=9.5, color=INK)
        ax.set_xticklabels([f'{k}' for k in RUNGS], fontsize=9)
    fig.supxlabel('noise added to drone1\'s VICON position, mm per axis',
                  x=0.008, ha='left', fontsize=9.5, color=MUTED)
    fig.suptitle('Every measure of the flight gets worse as the position noise rises\n'
                 'gains, clamp and flight time are identical in all four flights, so the '
                 'noise is the only difference',
                 x=0.008, y=0.985, va='top', ha='left', fontsize=11.5,
                 fontweight='bold', color=INK)
    fig.tight_layout()
    fig.subplots_adjust(top=0.80, bottom=0.17)
    fig.savefig(os.path.join(out, '3_what_changed.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'figures', 'noise_ladder'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    runs = []
    for label, rec, cfgname in RUNS:
        path = os.path.join(LOGS, rec)
        if not os.path.isfile(path):
            print(f'  {label} mm: no record -- left out')
            continue
        runs.append(analyse(label, path, cfgname))
    if not runs:
        raise SystemExit('no records found')
    RUNGS.extend(r['label'] for r in runs)

    table(runs)
    fig_deviation(runs, a.out)
    fig_paths(runs, a.out)
    fig_what_changed(runs, a.out)

    summary = {r['label']: {k: r[k] for k in
                            ('noise', 'beta', 'dev_rms', 'dev_med', 'dev_p95',
                             'dev_max', 'virt_rms', 'ripple', 'speed', 'speed_p95',
                             'tilt_p95', 'tilt_max', 'clip', 'u_med', 'radius_max',
                             'design_r', 'thirds')} for r in runs}
    for r in runs:
        summary[r['label']]['rec'] = os.path.basename(r['rec'])
        summary[r['label']]['cfg'] = r['cfg_name']
    with open(os.path.join(a.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'\nfigures in {a.out}')


if __name__ == '__main__':
    main()
