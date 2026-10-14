#!/usr/bin/env python3
"""The figures that go in the paper, authored at the size they print at.

    python3 tools/plot_paper.py          # -> docs/figures/paper/*.pdf and *.png

The index figures are built 9-12 in wide for a web page you can zoom. Dropped
into a 3.5 in column they shrink to about a third, which puts 9 pt labels at
3 pt. These are rebuilt instead of rescaled, and follow three rules:

  * NO titles or suptitles anywhere. The caption under the figure carries the
    description and the figure number; a title inside the axes duplicates it
    and costs vertical space.
  * One column (3.5 in) or full width (6.5 in), nothing in between, and
    authored at exactly that width.
  * Fewer panels. Anything that cannot be read at final size is cut rather
    than shrunk.

Page geometry is taken from the report: US Letter, 1 in margins, two 3.5 in
columns.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.lines import Line2D                               # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'docs', 'figures', 'paper')

COL, FULL = 3.5, 6.5          # inches, from the document's column setup
INK, MUTED, GRID, DESIGN = '#1f2328', '#5c6370', '#dfe2e6', '#9aa2ab'

# One accent per figure, not one per rung. In the index a colour ramp carries
# the rung identity because rungs share axes there; here each rung has its own
# row and its own label, so a ramp would encode nothing while costing contrast
# on the pale end and greyscale safety throughout. The accent only has to
# separate "flown" from the grey "design" line.
FLOWN = {'trochoidal': '#c2571a', 'coverage': '#1f5fbd',
         'flocking': '#157f58', 'distance': '#7a6414'}

RC = {
    'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8,
    'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 7,
    'axes.edgecolor': MUTED, 'axes.labelcolor': INK,
    'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.axisbelow': True, 'legend.frameon': False,
    'lines.linewidth': 1.0, 'savefig.dpi': 400,
    'figure.constrained_layout.use': True,
    'figure.constrained_layout.h_pad': 0.012,
    'figure.constrained_layout.w_pad': 0.012,
    'figure.constrained_layout.hspace': 0.02,
    'figure.constrained_layout.wspace': 0.02,
    'pdf.fonttype': 42, 'ps.fonttype': 42,
}


def save(fig, stem):
    os.makedirs(OUT, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(OUT, f'{stem}.{ext}'), bbox_inches='tight',
                    pad_inches=0.01)
    plt.close(fig)
    print(f'  {stem}.pdf / .png')


def tag(ax, text, loc='upper left'):
    """The rung label, inside the axes, where a title used to be."""
    x, y, ha, va = (0.03, 0.96, 'left', 'top')
    if loc == 'upper right':
        x, ha = 0.97, 'right'
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va,
            fontsize=7.5, fontweight='bold', color=INK,
            bbox=dict(fc='white', ec='none', alpha=0.75, pad=1.2))


# ---------------------------------------------------------------- noise ladders
def noise_box(algo, stem, ramp):
    """One panel: deviation per rung, with the sensor-only null.

    The index version pairs this with a 700-sample time series, which at
    3.5 in is an unreadable hairball. The box plot is what carries the result,
    so the time series is cut.
    """
    import plot_noise_ladder_algos as NL
    spec = NL.SPEC[algo]
    runs = []
    for label, rec, cfgname in spec['runs']:
        p = os.path.join(NL.LOGS, rec)
        if os.path.isfile(p):
            runs.append(NL.analyse(spec, label, p, cfgname))
    NL.add_nulls(runs)


    fig, ax = plt.subplots(figsize=(COL, 2.05))
    x = np.arange(len(runs))
    bx = ax.boxplot([r['dev'] * 100 for r in runs], positions=x, widths=0.6,
                    whis=(5, 95), patch_artist=True, showfliers=False,
                    medianprops=dict(color=INK, lw=1.2),
                    whiskerprops=dict(color=MUTED, lw=0.8),
                    capprops=dict(color=MUTED, lw=0.8))
    for patch, c in zip(bx['boxes'], ramp):
        patch.set_facecolor(c); patch.set_edgecolor('white'); patch.set_linewidth(1.1)
    ax.set_xticks(x, [f"{r['label']}" for r in runs])
    ax.set_xlabel('injected position noise (mm per axis)')
    ax.set_ylabel('deviation from the\ndesigned position (cm)')
    ax.set_ylim(bottom=0)
    save(fig, stem)
    for r in runs:
        print(f"      {r['label']:>2}mm  median {r['dev_med'] * 100:5.2f} cm")
    return {r['label']: r['dev_med'] * 100 for r in runs}


def trochoidal_noise_box(stem):
    """The 23 Sep trochoidal noise ladder, same single-panel treatment.

    That ladder lives in its own module (tools/plot_noise_ladder.py) and has
    no sensor-only null in it -- it predates the null model -- so the bars are
    shown without one rather than inventing a reference.
    """
    import plot_noise_ladder as N
    runs = []
    for label, rec, cfgname in N.RUNS:
        p = os.path.join(N.LOGS, rec)
        if os.path.isfile(p):
            runs.append(N.analyse(label, p, cfgname))
    ramp = ['#a3c9e6', '#5b9bd0', '#2a6ca8', '#0d3b63']

    fig, ax = plt.subplots(figsize=(COL, 2.05))
    x = np.arange(len(runs))
    bx = ax.boxplot([r['dev'] * 100 for r in runs], positions=x, widths=0.6,
                    whis=(5, 95), patch_artist=True, showfliers=False,
                    medianprops=dict(color=INK, lw=1.2),
                    whiskerprops=dict(color=MUTED, lw=0.8),
                    capprops=dict(color=MUTED, lw=0.8))
    for patch, c in zip(bx['boxes'], ramp):
        patch.set_facecolor(c); patch.set_edgecolor('white'); patch.set_linewidth(1.1)
    ax.set_xticks(x, [r['label'] for r in runs])
    ax.set_xlabel('injected position noise (mm per axis)')
    ax.set_ylabel('deviation from the\ndesigned pattern (cm)')
    ax.set_ylim(bottom=0)
    save(fig, stem)


def benchmarks(algo, stem, flight_keys=None, native_n=3):
    """Two rows: how the aircraft flew, and what the algorithm's metrics said.

    Full width, because the contrast only exists if both rows are visible at
    once. Four flight measures reduced to two -- deviation and tilt -- since
    oscillation and acceleration both sit at their nulls and say nothing.
    """
    import plot_noise_ladder_algos as NL
    spec = NL.SPEC[algo]
    runs = []
    for label, rec, cfgname in spec['runs']:
        p = os.path.join(NL.LOGS, rec)
        if os.path.isfile(p):
            runs.append(NL.analyse(spec, label, p, cfgname))
    ramp = NL.RAMP5 if len(runs) > 4 else NL.RAMP4
    x = np.arange(len(runs))

    flight = [('dev_rms', 100, 'deviation\nfrom design (cm)'),
              ('ripple', 100, 'oscillation (cm)'),
              ('tilt_med', 1, 'median tilt (deg)')]
    native = [(k, sc, lab) for k, sc, _u, lab, _f, _w, _x in spec['native']][:native_n]

    fig, axs = plt.subplots(2, 3, figsize=(FULL, 2.9))
    for ax, (key, sc, ylab) in zip(axs[0], flight):
        vals = [r[key] * sc for r in runs]
        ax.bar(x, vals, width=0.68, color=ramp)
        span = 100 * (max(vals) - min(vals)) / np.mean(vals)
        ax.annotate(f'spans {span:.0f}% of its mean', (0.5, 0.94),
                    xycoords='axes fraction', ha='center', va='top',
                    fontsize=7, color=MUTED)
        ax.set_xticks(x, [r['label'] for r in runs])
        ax.set_ylabel(ylab)
        ax.set_ylim(0, max(vals) * 1.28)
    short = {'cdist_mean': 'mean centroid\ndistance (cm)',
             'cdist_max': 'worst centroid\ndistance (cm)',
             'H': 'locational cost H',
             'edge_rms': 'edge-length\nerror (cm)',
             'shape_err': 'shape error after\nrigid fit (cm)',
             'W_lyap': 'Lyapunov\nfunction W'}
    for ax, (key, sc, lab) in zip(axs[1], native):
        vals = [r['nat'][key] * sc for r in runs]
        span = 100 * (max(vals) - min(vals)) / np.mean(vals)
        # Grey when the metric does not move, coloured when it tracks the
        # flight -- the contrast between the two rows is the whole figure.
        ax.bar(x, vals, width=0.68, color='#c4b8a8' if span < 15 else ramp)
        ax.set_xticks(x, [r['label'] for r in runs])
        ax.set_ylabel(short.get(key, lab.split('(')[0].strip().lower()))
        # Start at zero. These bars exist to show that nothing moves, and an
        # axis cropped to the data turns a 2% spread into a dramatic skyline.
        ax.set_ylim(0, max(vals) * 1.30)
        ax.annotate(f'spans {span:.0f}% of its mean',
                    (0.5, 0.94), xycoords='axes fraction',
                    ha='center', va='top', fontsize=7, color=MUTED)
    for ax in axs[1]:
        ax.set_xlabel('noise (mm per axis)')
    save(fig, stem)


# ------------------------------------------------------ design vs actual rows
def design_rows(algo, stem, rungs=None, span=20.0, height=None, colour=None):
    """One row per rung: flown path, and x over time against the design.

    Full width. No titles -- the rung label sits inside the left panel and the
    median gap inside the right one, which is where a reader looks anyway.
    """
    import plot_ladder as PL
    import ladder_common as C
    spec = PL.LADDERS[algo]
    runs = []
    for label, rec, cfgname in spec['runs']:
        if rungs and label not in rungs:
            continue
        r = PL.analyse(label, rec, cfgname, spec)
        if r:
            runs.append(r)
    runs.sort(key=lambda r: np.mean([d['k_tau'] for d in r['drones'].values()]))
    c = colour or FLOWN.get(algo, INK)

    # Square limits centred on the data, not on the origin. A formation that
    # holds station away from the centre (the octahedron's anchor sits at
    # x = 0.6) otherwise ends up as a small blob in the corner of a panel
    # sized for a pattern that is centred.
    pts = np.concatenate([r['P'][:, r['real'][0] if r['real'] else 0] for r in runs])
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    lim = float(np.abs(pts - [cx, cy]).max()) * 1.25 + 0.05
    h = height or (0.95 * len(runs) + 0.45)
    fig, axs = plt.subplots(len(runs), 2, figsize=(FULL, h),
                            gridspec_kw=dict(width_ratios=[1, 3.0]))
    axs = np.atleast_2d(axs)
    for row, r in zip(axs, runs):
        j = r['real'][0] if r['real'] else 0
        mid = 0.5 * r['t'][-1]
        t0, t1 = max(r['t'][0], mid - span / 2), min(r['t'][-1], mid + span / 2)
        m = (r['t'] >= t0) & (r['t'] <= t1)
        st = r['sim']['t'] - r['sim']['t'][0]
        want = np.stack([np.interp(r['t'][m] + 2.0, st, r['sim']['P'][:, j, ax])
                         for ax in (0, 1)], 1)
        act = r['P'][m, j]

        ax = row[0]
        ax.plot(want[:, 0], want[:, 1], color=DESIGN, lw=2.2, solid_capstyle='round')
        ax.plot(act[:, 0], act[:, 1], color=c, lw=0.8, alpha=0.9)
        ax.set_aspect('equal')
        ax.set_xlim(cx - lim, cx + lim); ax.set_ylim(cy - lim, cy + lim)
        ax.set_ylabel('y (m)')
        tag(ax, f"k·τ {PL.ktau(r):.2f}")

        ax = row[1]
        ax.plot(r['t'][m], want[:, 0], color=DESIGN, lw=2.2, solid_capstyle='round')
        ax.plot(r['t'][m], act[:, 0], color=c, lw=1.0, alpha=0.9)
        ax.set_xlim(t0, t1); ax.set_ylim(cx - lim, cx + lim)
        ax.set_ylabel('x (m)')
        gap = np.hypot(*(act - want).T) * 100
        tag(ax, f"median {np.median(gap):.0f} cm off", loc='upper right')
    for ax in axs[:-1].ravel():
        ax.set_xticklabels([])
    axs[-1, 0].set_xlabel('x (m)')
    axs[-1, 1].set_xlabel('time since the algorithm started (s)')
    fig.legend(handles=[Line2D([], [], color=DESIGN, lw=2.2, label='design'),
                        Line2D([], [], color=c, lw=1.0, label='flown')],
               loc='outside lower right', ncol=2, handlelength=1.5)
    save(fig, stem)


def trochoidal_rows(stem, rungs=('r2', 'r3', 'r4', 'r14'), secs=20.0):
    """The trochoidal equivalent, which has its own analysis module."""
    import plot_trochoidal_ladder as T
    runs = []
    for label, rec, cfgname in T.RUNS:
        if label not in rungs:
            continue
        p = os.path.join(T.LOGS, rec)
        if os.path.isfile(p):
            runs.append(T.analyse(label, p, cfgname))
    T.RUNGS.extend(r['label'] for r in runs)

    lim = 1.05
    c = FLOWN['trochoidal']
    fig, axs = plt.subplots(len(runs), 2, figsize=(FULL, 0.95 * len(runs) + 0.45),
                            gridspec_kw=dict(width_ratios=[1, 3.0]))
    axs = np.atleast_2d(axs)
    for row, r in zip(axs, runs):
        j = r['real'][0]
        a, b = 2.0, 2.0 + secs
        et, ev = r['start_tau'], r['start_P'][:, j]
        m = (r['tau'] >= a) & (r['tau'] <= b)
        ms = (et >= a) & (et <= b)
        act, exp_ = r['Pg'][m, j], ev[ms]

        ax = row[0]
        ax.plot(exp_[:, 0], exp_[:, 1], color=DESIGN, lw=2.2, solid_capstyle='round')
        ax.plot(act[:, 0], act[:, 1], color=c, lw=0.8, alpha=0.9)
        ax.set_aspect('equal')
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
        ax.set_ylabel('y (m)')
        tag(ax, f"β = {r['beta']:g}   k·τ {r['beta'] * T.TAU_NOMINAL:.2f}")

        ax = row[1]
        ax.plot(et[ms], exp_[:, 0], color=DESIGN, lw=2.2, solid_capstyle='round')
        ax.plot(r['tau'][m], act[:, 0], color=c, lw=1.0, alpha=0.9)
        ax.set_xlim(a, b); ax.set_ylim(-lim, lim)
        ax.set_ylabel('x (m)')
    for ax in axs[:-1].ravel():
        ax.set_xticklabels([])
    axs[-1, 0].set_xlabel('x (m)')
    axs[-1, 1].set_xlabel('time since the algorithm started (s)')
    fig.legend(handles=[Line2D([], [], color=DESIGN, lw=2.2, label='design'),
                        Line2D([], [], color=c, lw=1.0, label='flown')],
               loc='outside lower right', ncol=2, handlelength=1.5)
    save(fig, stem)


def main():
    plt.rcParams.update(RC)
    import plot_noise_ladder_algos as NL
    print('paper figures ->', OUT)
    trochoidal_rows('fig_trochoidal_design')
    trochoidal_noise_box('fig_noise_trochoidal_box')
    noise_box('coverage', 'fig_noise_coverage_box', NL.RAMP5)
    noise_box('flocking', 'fig_noise_flocking_box', NL.RAMP4)
    noise_box('kuramoto', 'fig_noise_kuramoto_box', NL.RAMP4)
    benchmarks('coverage', 'fig_coverage_benchmarks')
    benchmarks('distance', 'fig_distance_benchmarks')
    noise_box('distance', 'fig_noise_distance_box', NL.RAMP4)
    design_rows('distance', 'fig_distance_design')
    design_rows('coverage', 'fig_coverage_design')
    design_rows('flocking', 'fig_flocking_design')


if __name__ == '__main__':
    main()
