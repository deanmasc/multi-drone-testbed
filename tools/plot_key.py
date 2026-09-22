#!/usr/bin/env python3
"""The one figure the project is about: every flight, on one axis.

    python3 tools/plot_key.py        # -> docs/figures/key/1_threshold.png

Reads the three ladders' summary.json, so it cannot disagree with the per-ladder
figures. Run the ladder scripts first.
"""

import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402

import ladder_common as C                                        # noqa: E402

plt.rcParams.update(C.RC)
FIG = os.path.join(C.ROOT, 'docs', 'figures')

# Three categorical hues, validated all-pairs for normal and CVD vision
# (dataviz slots 1-3). Every point is also directly labelled, which is what the
# contrast warning on the aqua obliges.
ALGOS = [
    ('trochoidal_ladder', 'Trochoidal', '#2a78d6', 'o'),
    ('coverage_ladder', 'Coverage', '#eb6834', 's'),
    ('flocking_ladder', 'Flocking', '#1baf7a', '^'),
]
BAND = (0.67, 1.01)          # quiet below, sustained above -- see PROJECT_AIM 15f

# Rungs that would otherwise print on top of a neighbour or the band caption.
LABEL_XY = {'c3': (8, -11), 's1': (8, -4)}
LABEL_DEFAULT = (8, -3)
# Rungs that land on the floor of the linear axis: they are labelled in the
# inset instead, where there is room to tell them apart.
INSET_RUNGS = ('r1', 'c1', 's3', 'r2', 's2', 'c2')
INSET_XY = {'r1': (-6, -15), 'c1': (-4, 8), 's3': (3, 6), 'r2': (-20, -3),
            's2': (3, 6), 'c2': (5, 2)}


def points():
    for key, name, colour, marker in ALGOS:
        path = os.path.join(FIG, key, 'summary.json')
        if not os.path.exists(path):
            continue
        for rung, r in json.load(open(path)).items():
            for i, d in r.get('drones', {}).items():
                if not d.get('real'):
                    continue
                yield dict(algo=name, colour=colour, marker=marker, rung=rung,
                           drone=i, **d)


def main():
    out = os.path.join(FIG, 'key')
    os.makedirs(out, exist_ok=True)
    P = list(points())
    if not P:
        print('No summary.json files yet -- run the ladder scripts first.')
        return

    fig, ax = plt.subplots(1, 2, figsize=(11.6, 4.6))

    # -- the result ----------------------------------------------------------
    ax[0].axvspan(*BAND, color='#f0ead8', zorder=0)
    ax[0].annotate('the threshold\nis in here', (np.mean(BAND), 0.80),
                   xycoords=('data', 'axes fraction'), ha='center', va='top',
                   fontsize=8, color=C.MUTED)
    seen = set()
    for p in P:
        ax[0].plot(p['k_tau'], p['wobble'] * 100, p['marker'], ms=9,
                   color=p['colour'], zorder=3, alpha=0.95,
                   label=p['algo'] if p['algo'] not in seen else None)
        seen.add(p['algo'])
    done = set()
    for p in P:                                   # one label per rung
        if p['rung'] in done or p['rung'] in INSET_RUNGS:
            continue
        done.add(p['rung'])
        ax[0].annotate(p['rung'], (p['k_tau'], p['wobble'] * 100),
                       textcoords='offset points',
                       xytext=LABEL_XY.get(p['rung'], LABEL_DEFAULT),
                       fontsize=8, color=C.INK)
    ax[0].set_ylim(bottom=0)
    ax[0].set_xlabel('k · τ    (own-velocity gain × loop delay)')
    ax[0].set_ylabel('radius of oscillation (cm)')
    ax[0].set_title('One number predicts which flights oscillate')
    ax[0].legend(fontsize=8.5, loc='center right')

    # The linear axis is the point -- below the band everything is on the floor.
    # That also makes the five quiet rungs indistinguishable, so they get an
    # inset rather than labels that would sit on top of each other.
    iz = ax[0].inset_axes([0.46, 0.09, 0.50, 0.30])
    iz.axvspan(*BAND, color='#f0ead8', zorder=0)
    for p in P:
        if p['rung'] not in INSET_RUNGS:
            continue
        iz.plot(p['k_tau'], p['wobble'] * 100, p['marker'], ms=7,
                color=p['colour'], zorder=3, alpha=0.95)
    for rung in INSET_RUNGS:
        q = next((p for p in P if p['rung'] == rung), None)
        if q is None:
            continue
        iz.annotate(rung, (q['k_tau'], q['wobble'] * 100),
                    textcoords='offset points', xytext=INSET_XY[rung],
                    fontsize=7.5, color=C.INK)
    iz.set_xlim(0.21, 0.80); iz.set_ylim(-0.35, 3.1)
    iz.tick_params(labelsize=7)
    iz.set_title('the quiet end, magnified  (0–3 cm)', fontsize=8,
                 color=C.MUTED, pad=3)

    # -- the mechanism -------------------------------------------------------
    lim = [0.4, 1.8]
    ax[1].plot(lim, lim, '--', color=C.DESIGN, lw=1.2, zorder=1)
    ax[1].annotate('period = 4τ', (1.5, 1.57), rotation=38, fontsize=8,
                   color=C.MUTED)
    for p in P:
        if p.get('xc_max', 1.0) < 0.5 or not p.get('period'):
            continue                              # nothing ringing to time
        ax[1].plot(4 * p['lag'], p['period'], p['marker'], ms=9,
                   color=p['colour'], zorder=3, alpha=0.95)
    ax[1].set_xlim(lim); ax[1].set_ylim(lim)
    ax[1].set_xlabel('4 × measured loop delay (s)')
    ax[1].set_ylabel('measured oscillation period (s)')
    ax[1].set_title('When it does oscillate, the delay sets the period')

    n = len({p['rung'] for p in P})
    WORD = {8: 'Eight', 9: 'Nine', 10: 'Ten', 11: 'Eleven', 12: 'Twelve',
            13: 'Thirteen', 14: 'Fourteen', 15: 'Fifteen', 16: 'Sixteen'}
    fig.suptitle(f'{WORD.get(n, n)} flights, three algorithms, three papers',
                 y=1.02, fontsize=12, fontweight='bold')
    fig.savefig(os.path.join(out, '1_threshold.png'))
    plt.close(fig)
    print(f'{len(P)} flight-drones -> {out}/1_threshold.png')


if __name__ == '__main__':
    main()
