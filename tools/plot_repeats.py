#!/usr/bin/env python3
"""The only two conditions in the corpus flown twice.

    python3 tools/plot_repeats.py        # -> docs/figures/repeatability/

Dean flagged two rungs as probably faulty: coverage 2 mm read higher than
5 mm, and the flocking 0 mm control read higher than its 2 mm rung. Each was
reflown once on 7 Oct, with the same config, the same marks and the same
analysis window as the 29 Sep original.

Every other number in this project is a single flight, so this is the first
and only direct measurement of how much a condition moves between sessions.
That matters more than whichever way the two rungs landed.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_noise_ladder_algos as NL                             # noqa: E402

ROOT = NL.ROOT
INK, MUTED, GRID = NL.INK, NL.MUTED, NL.GRID

# (algorithm, hue, the rung flown twice, every rung's records)
CASES = [
    dict(algo='coverage', colour='#2a78d6', repeated='2',
         runs={'0':  ['coverage_20260929_151743.txt'],
               '2':  ['coverage_20260929_152418.txt', 'coverage_20261007_160703.txt'],
               '5':  ['coverage_20260929_152810.txt'],
               '10': ['coverage_20260929_153141.txt'],
               '20': ['coverage_20260929_153520.txt']}),
    dict(algo='flocking', colour='#1baf7a', repeated='0',
         runs={'0':  ['flocking_20260929_143348.txt', 'flocking_20261007_161108.txt'],
               '2':  ['flocking_20260929_144718.txt'],
               '5':  ['flocking_20260929_145234.txt'],
               '10': ['flocking_20260929_150050.txt']}),
]

plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
    'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': MUTED,
    'ytick.color': MUTED, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.6, 'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'axes.axisbelow': True, 'savefig.dpi': 170,
    'savefig.bbox': 'tight',
})


def measure(case):
    """RMS deviation and its sensor-only null, per rung, per flight."""
    spec = NL.SPEC[case['algo']]
    out = {}
    for rung, recs in case['runs'].items():
        cfgname = next(c for l, _, c in spec['runs'] if l == rung)
        vals = []
        for rec in recs:
            path = os.path.join(NL.LOGS, rec)
            if not os.path.isfile(path):
                print(f'  [skip] {rec}')
                continue
            r = NL.analyse(spec, rung, path, cfgname)
            vals.append((rec, r['dev_rms'] * 100))
        out[rung] = vals
    return out


def main():
    out_dir = os.path.join(ROOT, 'docs', 'figures', 'repeatability')
    os.makedirs(out_dir, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.1))

    for ax, case in zip(axes, CASES):
        data = measure(case)
        rungs = sorted(data, key=float)
        x = np.arange(len(rungs))
        for k, rung in enumerate(rungs):
            vals = [v for _, v in data[rung]]
            if not vals:
                continue
            if len(vals) == 1:
                ax.plot(k, vals[0], 'o', ms=8, color=case['colour'], zorder=3)
            else:
                ax.plot([k, k], [min(vals), max(vals)], '-', lw=2.4,
                        color=case['colour'], alpha=0.45, zorder=2,
                        solid_capstyle='round')
                ax.plot([k] * len(vals), vals, 'o', ms=9, color=case['colour'],
                        mec='white', mew=1.4, zorder=4)
                lo, hi = min(vals), max(vals)
                ax.annotate(f'two flights\nspan {hi - lo:.1f} cm',
                            (k, hi), textcoords='offset points',
                            xytext=(12, 2), fontsize=8, color=INK)
        ax.set_xticks(x, [f'{r} mm' for r in rungs])
        ax.set_ylim(bottom=0)
        ax.set_xlim(-0.5, len(rungs) - 0.3)
        ax.set_ylabel('deviation from the designed\npattern, RMS (cm)')
        n = len(data[case['repeated']])
        ax.set_title(f"{case['algo'].capitalize()} — {case['repeated']} mm "
                     f"flown {'twice' if n == 2 else f'{n}x'}")
    fig.suptitle('The only two conditions flown more than once',
                 y=1.06, x=0.008, ha='left', fontsize=12, fontweight='bold')
    fig.text(0.008, 0.99,
             'Dean flagged both as probably faulty. Each was reflown once on '
             '7 Oct with the same config, marks and analysis window as the '
             '29 Sep original.',
             ha='left', fontsize=8.8, color=MUTED)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '1_repeats.png'))
    plt.close(fig)

    print('\nRMS deviation per flight (cm)')
    for case in CASES:
        print(f"\n{case['algo']}:")
        for rung, vals in sorted(measure(case).items(), key=lambda kv: float(kv[0])):
            for rec, v in vals:
                mark = '  <- repeat' if '20261007' in rec else ''
                print(f'   {rung:>3} mm  {v:6.2f}   {rec}{mark}')
    print(f'\nfigure -> {out_dir}/1_repeats.png')


if __name__ == '__main__':
    main()
