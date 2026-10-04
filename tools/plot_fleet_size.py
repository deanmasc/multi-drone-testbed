#!/usr/bin/env python3
"""Fleet size against measurement noise: one real drone vs two (23 + 30 Sep 2026).

A 2x2. Trochoidal consensus at beta = 2 (k*tau 0.56), 0 and 10 mm of injected
VICON noise per axis, flown once with drone1 real and once with drone1 AND
drone4 real. The question is whether the gap to simulation widens when more of
the fleet is a real aircraft, at the same noise level.

    python3 tools/plot_fleet_size.py

READ THIS BEFORE QUOTING THE NUMBERS. Two things confound the comparison and
neither can be fixed in analysis:

  1 NOT QUITE THE SAME PATTERN. On 23 Sep the real drone1 sat on drone4's floor
    mark, 41 cm from its own; on 30 Sep drone1 was there again and drone4 took
    the vacant mark, so the pair was exchanged. Each start produces its own
    trochoid. Deviation is measured against a noiseless replay seeded from each
    run's OWN engage positions, so it is always "how far from where the law
    wanted it" -- and measured, the four designed patterns turn out close:
    radius 24 / 21 / 20 / 20 cm and design speed 0.025 / 0.023 / 0.021 /
    0.021 m/s, a spread of about 15%. Small enough that the x1.7-x2.3 effects
    below are not explained by it, large enough to report. Both panels are on
    the right of 3_caveats.png.

  2 ONE FLIGHT PER CELL, and the two fleet sizes are a week apart.

  3 THE RECORDED x,y IS THE NOISY READING (metrics_recorder takes it from
    /state). Every position-derived bar therefore carries a sensor-only null:
    that cell's own 0 mm trajectory re-measured through 10 mm of noise. Tilt
    comes from /poses and is clean.

So: directional evidence that fleet size matters, not a measured coefficient.
"""

import argparse
import copy
import json
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.lines import Line2D                             # noqa: E402
from matplotlib.patches import Patch                            # noqa: E402
from scipy.signal import butter, sosfiltfilt                    # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyse_trochoidal as A                                  # noqa: E402
import sim_baseline as S                                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed', 'config')
LOGS = os.path.join(ROOT, 'logs', 'hw')

DT = 0.1
WIN_SKIP = 2.0
# Set by the 30 Sep flights: the 0 mm one was stopped early when a drone
# misbehaved and the 10 mm one ran the battery flat at ~110 s. Both fleet
# sizes are cut to the same 93 s so the distributions cover the same ground.
WIN = 93.0
SHAKE = (0.6, 1.5)

# (fleet label, noise mm, record, config, which agents were real)
RUNS = [
    ('1 real', '0',  'trochoidalconsensus_20260923_152619.txt',
     'testbed_fig4_r2_n0.yaml'),
    ('1 real', '10', 'trochoidalconsensus_20260923_155129.txt',
     'testbed_fig4_r2_n10.yaml'),
    ('2 real', '0',  'trochoidalconsensus_20260930_151018.txt',
     'testbed_fig4_r2_n0.yaml'),
    ('2 real', '10', 'trochoidalconsensus_20260930_160005.txt',
     'testbed_fig4_r2_n10.yaml'),
]

INK = '#1f2328'
MUTED = '#6b7280'
GRID = '#e5e7eb'
DESIGN = '#9ca3af'
# Fleet size is an identity, not a magnitude, so it gets two categorical hues
# rather than a ramp. Blue/orange, the standard colour-vision-safe pair, and
# distinct from the noise ladders' single cool ramp.
FLEET = {'1 real': '#2a6ca8', '2 real': '#c2571a'}
NOISEPOS = {'0': 0, '10': 1}


def bp(x):
    return sosfiltfilt(butter(4, list(SHAKE), 'band', fs=1 / DT, output='sos'), x)


def read_real(path, ids):
    import re
    for line in open(path):
        if not line.startswith('#'):
            break
        m = re.search(r'real drones\s+(.*)', line)
        if m:
            got = tuple(p.split('=')[0].strip() for p in m.group(1).split(','))
            got = tuple(i for i in got if i in ids)
            if got:
                return got
    return ('drone1',)


def analyse(fleet, noise, rec, cfgname):
    cfg = yaml.safe_load(open(os.path.join(CFG, cfgname)))
    ids = [d['id'] for d in cfg['drones']]
    path = os.path.join(LOGS, rec)
    cols, D = A.load_record(path)
    t = D[:, 0]
    P = np.stack([np.stack([D[:, cols.index(f'x_{i}')], D[:, cols.index(f'y_{i}')]], 1)
                  for i in ids], 1)
    last = A.live_end(P)
    _, _, _, _, t_start = A.classify(t, P, last, ids)
    reals = read_real(path, ids)

    tg = np.arange(t_start + WIN_SKIP, t_start + WIN_SKIP + WIN, DT)
    if tg[-1] > t[last]:
        raise SystemExit(f'{fleet} {noise} mm: record ends at {t[last]:.0f} s, '
                         f'before the window closes at {tg[-1]:.0f} s')
    Pg = np.stack([np.stack([np.interp(tg, t, P[:, n, ax]) for ax in (0, 1)], 1)
                   for n in range(len(ids))], 1)

    ks = int(np.searchsorted(t, t_start - 0.5))
    placed = {i: float(np.hypot(*(P[0, n] - np.array(cfg['drones'][n]['initial_position'][:2]))))
              for n, i in enumerate(ids) if i in reals}
    cfg0 = copy.deepcopy(cfg)
    for n, d in enumerate(cfg0['drones']):
        d['initial_position'] = P[ks, n].tolist()
        d['initial_velocity'] = [0.0, 0.0]
    _, ts0, _, ph0, _ = S.run(cfg0, tg[-1] - t_start + 2.0)
    Ps0 = np.array(ph0)
    Dg = np.stack([np.stack([np.interp(tg - t_start, ts0, Ps0[:, n, ax])
                             for ax in (0, 1)], 1) for n in range(len(ids))], 1)

    vj = [n for n, i in enumerate(ids) if i not in reals]
    per_real = {i: np.hypot(*(Pg[:, ids.index(i)] - Dg[:, ids.index(i)]).T) for i in reals}
    dev_virt = np.mean([np.hypot(*(Pg[:, n] - Dg[:, n]).T) for n in vj], axis=0)
    j = ids.index(reals[0])
    z = Pg[:, j, 0] + 1j * Pg[:, j, 1]
    tilt = np.interp(tg, t, D[:, cols.index(f'tilt_{reals[0]}')])
    sep = np.hypot(*(Pg[:, 0] - Pg[:, 3]).T)

    # how different is the pattern each fleet size was actually tracking?
    dspeed = np.median(np.hypot(*np.gradient(Dg[:, j], DT, axis=0).T))
    return dict(
        fleet=fleet, noise=noise, rec=rec, cfg_name=cfgname, reals=reals,
        ids=ids, j=j, tau=tg - t_start, Pg=Pg, Dg=Dg, placed=placed,
        dev=per_real[reals[0]], dev_vec=Pg[:, j] - Dg[:, j],
        per_real={i: float(np.sqrt(np.mean(v ** 2))) for i, v in per_real.items()},
        dev_rms=float(np.sqrt(np.mean(per_real[reals[0]] ** 2))),
        dev_med=float(np.median(per_real[reals[0]])),
        dev_p95=float(np.percentile(per_real[reals[0]], 95)),
        dev_max=float(per_real[reals[0]].max()),
        virt_rms=float(np.sqrt(np.mean(dev_virt ** 2))),
        ripple=float(np.sqrt(np.mean(np.abs(bp(z.real) + 1j * bp(z.imag)) ** 2))),
        tilt_med=float(np.nanmedian(tilt)), tilt_p95=float(np.nanpercentile(tilt, 95)),
        sep_min=float(sep.min()), sep_med=float(np.median(sep)),
        design_r=float(np.median(np.hypot(*Dg[:, j].T))), design_speed=float(dspeed),
        radius_max=float(np.hypot(*Pg[:, j].T).max()),
    )


def add_nulls(runs, draws=200, seed=0):
    """Each cell's own 0 mm run, re-measured through that cell's noise."""
    rng = np.random.default_rng(seed)
    base = {r['fleet']: r for r in runs if r['noise'] == '0'}
    for r in runs:
        b = base[r['fleet']]
        s = float(r['noise']) / 1000.0
        if s <= 0:
            r['dev_rms_null'] = b['dev_rms']
            r['ripple_null'] = b['ripple']
            r['virt_null'] = b['virt_rms']
            continue
        dv, rp = [], []
        P0 = b['Pg'][:, b['j']]
        for _ in range(draws):
            e = rng.normal(0.0, s, P0.shape)
            dv.append(np.sqrt(np.mean(np.hypot(*(b['dev_vec'] + e).T) ** 2)))
            rp.append(np.sqrt(np.mean(np.abs(bp(P0[:, 0] + e[:, 0])
                                             + 1j * bp(P0[:, 1] + e[:, 1])) ** 2)))
        r['dev_rms_null'] = float(np.mean(dv))
        r['ripple_null'] = float(np.mean(rp))
        # the simulated agents never carry injected noise themselves
        r['virt_null'] = b['virt_rms']


def table(runs):
    print(f'\nfleet size x measurement noise, trochoidal beta = 2 (k*tau 0.56), '
          f'{WIN:.0f} s window from engage + {WIN_SKIP:.0f} s\n')
    print(f'{"fleet":>7} {"noise":>6} | {"deviation of the primary real drone (cm)":>44} | '
          f'{"sim":>11} | {"ripple":>13} | {"tilt deg":>12}')
    print(f'{"":>7} {"":>6} | {"RMS":>6} {"(null)":>7} {"median":>7} {"p95":>6} {"max":>6} '
          f'{"vs 1 real":>8} | {"agents":>11} | {"cm":>6} {"(null)":>6} | {"med":>5} {"p95":>5}')
    base = {r['fleet']: r for r in runs}
    for r in runs:
        one = [q for q in runs if q['fleet'] == '1 real' and q['noise'] == r['noise']][0]
        ratio = r['dev_rms'] / one['dev_rms']
        print(f"{r['fleet']:>7} {r['noise'] + 'mm':>6} | {r['dev_rms'] * 100:6.1f} "
              f"{r['dev_rms_null'] * 100:7.1f} {r['dev_med'] * 100:7.1f} "
              f"{r['dev_p95'] * 100:6.1f} {r['dev_max'] * 100:6.1f} "
              f"{('x%.2f' % ratio):>8} | {r['virt_rms'] * 100:6.1f} cm | "
              f"{r['ripple'] * 100:6.2f} {r['ripple_null'] * 100:6.2f} | "
              f"{r['tilt_med']:5.1f} {r['tilt_p95']:5.1f}")

    print('\nthe comparison the test exists for -- same noise, one real drone vs two:')
    for nz in ('0', '10'):
        a = [r for r in runs if r['fleet'] == '1 real' and r['noise'] == nz][0]
        b = [r for r in runs if r['fleet'] == '2 real' and r['noise'] == nz][0]
        print(f"  {nz:>2} mm   deviation {a['dev_rms'] * 100:5.1f} -> {b['dev_rms'] * 100:5.1f} cm "
              f"(x{b['dev_rms'] / a['dev_rms']:.2f}),  "
              f"simulated agents {a['virt_rms'] * 100:4.1f} -> {b['virt_rms'] * 100:4.1f} "
              f"(x{b['virt_rms'] / a['virt_rms']:.2f}),  "
              f"median tilt {a['tilt_med']:.1f} -> {b['tilt_med']:.1f} deg "
              f"(x{b['tilt_med'] / a['tilt_med']:.2f})")

    print('\nper real drone, RMS deviation (cm):')
    for r in runs:
        print(f"  {r['fleet']:>7} {r['noise']:>3}mm   " +
              '   '.join(f'{i} {v * 100:5.1f}' for i, v in r['per_real'].items()))

    print('\nHOW DIFFERENT WERE THE PATTERNS? (the confound -- see the module docstring)')
    print(f'{"fleet":>7} {"noise":>6} {"design radius":>14} {"design speed":>14} '
          f'{"drone1 off its mark at placement":>34}')
    for r in runs:
        off = '   '.join(f'{i} {v * 100:.0f} cm' for i, v in r['placed'].items())
        print(f"{r['fleet']:>7} {r['noise'] + 'mm':>6} {r['design_r'] * 100:11.1f} cm "
              f"{r['design_speed']:11.3f} m/s   {off:>34}")
    print('\nd1-d4 separation actually flown (m):')
    for r in runs:
        kind = 'both real' if len(r['reals']) > 1 else 'real vs simulated'
        print(f"  {r['fleet']:>7} {r['noise']:>3}mm  min {r['sep_min']:.2f}  "
              f"median {r['sep_med']:.2f}   ({kind})")


def _cells(runs):
    return {(r['fleet'], r['noise']): r for r in runs}


def fig_deviation(runs, out):
    """Deviation over time for all four cells, and the 2x2 as paired bars."""
    fig, axs = plt.subplots(1, 2, figsize=(12.6, 4.8),
                            gridspec_kw=dict(width_ratios=[2.1, 1]))
    ax = axs[0]
    for r in runs:
        ax.plot(r['tau'], r['dev'] * 100, color=FLEET[r['fleet']],
                lw=1.5 if r['noise'] == '10' else 1.0,
                ls='-' if r['noise'] == '10' else (0, (4, 2)),
                alpha=0.95)
    ax.set_xlim(0, WIN)
    ax.set_ylim(0, max(r['dev_max'] for r in runs) * 100 * 1.25)
    ax.set_xlabel('seconds since the algorithm started')
    ax.set_ylabel('distance from the designed position (cm)')
    ax.set_title('Deviation of the primary real drone, all four flights',
                 loc='left', fontsize=10)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    h = [Line2D([], [], color=FLEET['1 real'], lw=1.5),
         Line2D([], [], color=FLEET['2 real'], lw=1.5),
         Line2D([], [], color=MUTED, lw=1.0, ls=(0, (4, 2))),
         Line2D([], [], color=MUTED, lw=1.5)]
    ax.legend(h, ['one real drone', 'two real drones', 'no added noise', '10 mm noise'],
              fontsize=8.5, ncol=2, loc='upper left', framealpha=0.9)

    ax = axs[1]
    C = _cells(runs)
    w = 0.34
    for n, fleet in enumerate(('1 real', '2 real')):
        xs = [NOISEPOS[nz] + (n - 0.5) * w for nz in ('0', '10')]
        vals = [C[(fleet, nz)]['dev_rms'] * 100 for nz in ('0', '10')]
        ax.bar(xs, vals, width=w * 0.92, color=FLEET[fleet], label=fleet)
        for x, v in zip(xs, vals):
            ax.annotate(f'{v:.1f}', (x, v), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=9, color=INK)
    for nz in ('0', '10'):
        nl = C[('2 real', nz)]['dev_rms_null'] * 100
        ax.plot([NOISEPOS[nz] + 0.5 * w - w * 0.46, NOISEPOS[nz] + 0.5 * w + w * 0.46],
                [nl, nl], color=INK, lw=1.4, zorder=4)
        nl = C[('1 real', nz)]['dev_rms_null'] * 100
        ax.plot([NOISEPOS[nz] - 0.5 * w - w * 0.46, NOISEPOS[nz] - 0.5 * w + w * 0.46],
                [nl, nl], color=INK, lw=1.4, zorder=4)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['no added noise', '10 mm per axis'], fontsize=9)
    ax.set_xlim(-0.55, 1.55)
    ax.set_ylabel('deviation, RMS (cm)')
    ax.set_title('The 2x2  (black tick = sensor-only null)', loc='left', fontsize=10)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(fontsize=9, frameon=False, loc='upper left')

    fig.suptitle('At the same noise level, a second real aircraft widens the gap '
                 'to simulation',
                 x=0.008, y=0.985, va='top', ha='left', fontsize=12,
                 fontweight='bold', color=INK)
    fig.text(0.008, 0.925,
             'Trochoidal consensus, beta = 2, 93 s window, one flight per cell. '
             'Black tick is the sensor-only null.',
             va='top', ha='left', fontsize=9, color=MUTED)
    fig.tight_layout()
    fig.subplots_adjust(top=0.82)
    fig.savefig(os.path.join(out, '1_deviation.png'), dpi=150)
    plt.close(fig)


def fig_paths(runs, out):
    """The flown path against the designed one, laid out as the 2x2."""
    fig, axs = plt.subplots(2, 2, figsize=(8.6, 8.8))
    lim = max(max(np.abs(r['Pg'][:, r['j']]).max(), np.abs(r['Dg'][:, r['j']]).max())
              for r in runs) * 1.12
    C = _cells(runs)
    for row, fleet in enumerate(('1 real', '2 real')):
        for col, nz in enumerate(('0', '10')):
            r = C[(fleet, nz)]
            ax = axs[row][col]
            j = r['j']
            ax.plot(r['Dg'][:, j, 0], r['Dg'][:, j, 1], color=DESIGN, lw=3.0,
                    solid_capstyle='round')
            ax.plot(r['Pg'][:, j, 0], r['Pg'][:, j, 1], color=FLEET[fleet],
                    lw=0.9, alpha=0.9)
            if len(r['reals']) > 1:
                k = r['ids'].index(r['reals'][1])
                ax.plot(r['Pg'][:, k, 0], r['Pg'][:, k, 1], color=FLEET[fleet],
                        lw=0.7, alpha=0.4)
            ax.set_aspect('equal')
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.grid(color=GRID, lw=0.7)
            ax.set_axisbelow(True)
            for s in ('top', 'right'):
                ax.spines[s].set_visible(False)
            noise_txt = 'no added noise' if nz == '0' else '10 mm per axis'
            ax.set_title(f"{fleet}, {noise_txt}\nRMS deviation {r['dev_rms'] * 100:.1f} cm, "
                         f"designed radius {r['design_r'] * 100:.0f} cm",
                         loc='left', fontsize=9.5, color=INK)
            if row == 1:
                ax.set_xlabel('x, m')
            if col == 0:
                ax.set_ylabel('y, m')
    h = [Line2D([], [], color=DESIGN, lw=3.0),
         Line2D([], [], color=FLEET['1 real'], lw=1.2),
         Line2D([], [], color=FLEET['2 real'], lw=1.2),
         Line2D([], [], color=FLEET['2 real'], lw=1.2, alpha=0.4)]
    fig.legend(h, ['designed pattern (noiseless replay of that run\'s own start)',
                   'drone1 flown, one real drone', 'drone1 flown, two real drones',
                   'drone4 flown (second real aircraft)'],
               loc='lower center', ncol=2, fontsize=8.5, frameon=False,
               bbox_to_anchor=(0.5, 0.0))
    fig.suptitle('The flown path against the designed one, across the 2x2',
                 x=0.008, y=0.99, va='top', ha='left', fontsize=12,
                 fontweight='bold', color=INK)
    fig.text(0.008, 0.955,
             'Grey is each run\'s own noiseless replay. The four designed patterns are '
             'within 15% of each other on radius and speed.',
             va='top', ha='left', fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0.075, 1, 1))
    fig.subplots_adjust(top=0.88)
    fig.savefig(os.path.join(out, '2_paths.png'), dpi=150)
    plt.close(fig)


def fig_measures(runs, out):
    """Four measures as the same 2x2, with the confound shown beside them."""
    panels = [
        ('dev_rms', 100, 'cm', 'Deviation, primary real drone', '{:.1f}', 'dev_rms_null'),
        ('virt_rms', 100, 'cm', 'Deviation, the SIMULATED agents', '{:.1f}', 'virt_null'),
        ('ripple', 100, 'cm', f'Oscillation, {SHAKE[0]}-{SHAKE[1]} Hz', '{:.2f}', 'ripple_null'),
        ('tilt_med', 1, 'deg', 'Tilt, median (clean /poses channel)', '{:.1f}', None),
    ]
    confound = [('design_r', 100, 'cm', 'Designed pattern radius', '{:.0f}'),
                ('design_speed', 1, 'm/s', 'Designed pattern speed', '{:.3f}')]
    # Printed on the two grey panels instead of a ratio: how much of a confound
    # the different start positions actually turned out to be.
    fig, axs = plt.subplots(1, 6, figsize=(17.4, 3.9))
    C = _cells(runs)
    w = 0.34

    def grouped(ax, key, sc, fmt, nullkey=None, flat=False):
        top = 0
        for n, fleet in enumerate(('1 real', '2 real')):
            xs = [NOISEPOS[nz] + (n - 0.5) * w for nz in ('0', '10')]
            vals = [C[(fleet, nz)][key] * sc for nz in ('0', '10')]
            ax.bar(xs, vals, width=w * 0.92,
                   color=(MUTED if flat else FLEET[fleet]), label=fleet)
            for x, v in zip(xs, vals):
                ax.annotate(fmt.format(v), (x, v), textcoords='offset points',
                            xytext=(0, 3), ha='center', fontsize=8, color=INK)
            top = max(top, max(vals))
            if nullkey:
                for x, nz in zip(xs, ('0', '10')):
                    nv = C[(fleet, nz)][nullkey] * sc
                    ax.plot([x - w * 0.46, x + w * 0.46], [nv, nv],
                            color=INK, lw=1.3, zorder=4)
                    top = max(top, nv)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['0 mm', '10 mm'], fontsize=9)
        ax.set_xlim(-0.55, 1.55)
        ax.set_ylim(0, top * 1.26)
        ax.grid(axis='y', color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)

    for ax, (key, sc, unit, title, fmt, nullkey) in zip(axs[:4], panels):
        grouped(ax, key, sc, fmt, nullkey)
        ax.set_ylabel(unit)
        ax.set_title(title, loc='left', fontsize=9.5, color=INK, pad=16)
        g0 = C[('2 real', '0')][key] / C[('1 real', '0')][key]
        g10 = C[('2 real', '10')][key] / C[('1 real', '10')][key]
        ax.annotate(f'1->2 real: x{g0:.2f} at 0 mm, x{g10:.2f} at 10 mm',
                    (0.0, 1.012), xycoords='axes fraction', ha='left', va='bottom',
                    fontsize=8, color=INK)

    for ax, (key, sc, unit, title, fmt) in zip(axs[4:], confound):
        grouped(ax, key, sc, fmt, None, flat=True)
        ax.set_ylabel(unit)
        ax.set_title(title, loc='left', fontsize=9.5, color=INK, pad=16)
        vv = [C[(f, n)][key] * sc for f in ('1 real', '2 real') for n in ('0', '10')]
        ax.annotate(f'the confound: spans {(max(vv) - min(vv)) / np.mean(vv) * 100:.0f}% '
                    f'across the 2x2', (0.0, 1.012), xycoords='axes fraction',
                    ha='left', va='bottom', fontsize=8, color='#b4531a')

    axs[0].legend(fontsize=8.5, frameon=False, loc='upper left')
    fig.supxlabel('noise added to each real drone\'s VICON position, mm per axis',
                  x=0.008, ha='left', fontsize=9.5, color=MUTED)
    fig.suptitle('One real drone against two, at each noise level',
                 x=0.008, y=0.985, va='top', ha='left', fontsize=12,
                 fontweight='bold', color=INK)
    fig.text(0.008, 0.925,
             'Black ticks are the sensor-only null: that cell\'s own 0 mm flight '
             're-measured through 10 mm of noise, since the recorder takes x and y from '
             '/state. Tilt is from /poses and is clean.\nThe two grey panels are not '
             'results -- they are how far apart the four designed patterns were, and the '
             'answer is about 15%, too little to account for the effects on the left.',
             va='top', ha='left', fontsize=9, color=MUTED, linespacing=1.5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.74, bottom=0.17)
    fig.savefig(os.path.join(out, '3_caveats.png'), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'figures', 'fleet_size'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = [analyse(*r) for r in RUNS]
    add_nulls(runs)
    table(runs)
    fig_deviation(runs, a.out)
    fig_paths(runs, a.out)
    fig_measures(runs, a.out)
    keys = ('dev_rms', 'dev_rms_null', 'dev_med', 'dev_p95', 'dev_max', 'virt_rms',
            'virt_null', 'ripple', 'ripple_null', 'tilt_med', 'tilt_p95', 'sep_min',
            'sep_med', 'design_r', 'design_speed', 'radius_max')
    summary = {f"{r['fleet'].replace(' ', '')}_{r['noise']}mm":
               dict({k: r[k] for k in keys}, rec=os.path.basename(r['rec']),
                    cfg=r['cfg_name'], reals=list(r['reals']),
                    per_real=r['per_real'], placed=r['placed'], window_s=WIN)
               for r in runs}
    with open(os.path.join(a.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'\nfigures in {a.out}')


if __name__ == '__main__':
    main()
