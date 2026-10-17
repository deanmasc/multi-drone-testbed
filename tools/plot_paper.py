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
         'flocking': '#157f58', 'distance': '#7a6414',
         'kuramoto': '#8c3b6d'}

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
    # KuramotoMetrics writes columns() rather than all_columns(), so its
    # records carry no z or tilt at all and the clean channel does not exist
    # for that law. Drop a measure that is NaN everywhere rather than plot an
    # empty panel.
    flight = [f for f in flight
              if np.isfinite([r.get(f[0], np.nan) for r in runs]).any()]
    # SPEC's last field is a warning string, non-empty only where the metric
    # is an artefact of the noisy channel (flocking's vel_spread reads the
    # least-squares velocity, where the fit has already multiplied position
    # noise by 11). A coloured bar that rises with the noise would read as
    # "this metric sees the gap" when what it sees is the injection, so the
    # metric is left out rather than plotted with a caveat.
    native = [(k, sc, lab) for k, sc, _u, lab, _f, _w, warn in spec['native']
              if not warn][:native_n]

    ncol = max(len(flight), len(native))
    fig, axs = plt.subplots(2, ncol, figsize=(FULL, 2.9), squeeze=False)
    for ax in axs.ravel()[len(flight):ncol]:
        ax.axis('off')
    for ax in axs[1][len(native):]:
        ax.axis('off')
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
             'W_lyap': 'Lyapunov\nfunction W',
             'order_R': 'order parameter R\n(1 = in phase)',
             'radius_rms': 'radial error\noff the ring (cm)',
             'angular_spacing_rms': 'angular spacing\nerror (rad)',
             'lattice_err': 'lattice error (cm)',
             'vel_spread': 'velocity spread (cm/s)',
             'd_min': 'smallest pair\ngap (cm)',
             'connected': 'graph connected\n(% of ticks)'}
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
    for ax in axs[1][:len(native)]:
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


# ------------------------------------------------- the promised property
PROMISE_LABEL = {'H': 'locational cost H',
                 'lattice_err': 'lattice error (m)',
                 'order_R': 'order parameter R',
                 'edge_rms': 'edge-length error (m)'}


def promise(algo, stem, height=1.85, colour=None, zero=True, t0=0.0):
    """One panel per rung: the quantity the source paper states its result in.

    The index version also draws the metric as the recorder logged it, on the
    recorder's own clock, which on coverage is a sawtooth four times the size
    of the real curve (PROJECT_AIM §15d). It is there to document the defect,
    and it sets the y-axis for every panel, so the curve the figure is
    actually about ends up in the bottom fifth. The paper figure drops it and
    lets the axis belong to the result.
    """
    import plot_ladder as PL
    spec = PL.LADDERS[algo]
    runs = []
    for label, rec, cfgname in spec['runs']:
        r = PL.analyse(label, rec, cfgname, spec)
        if r:
            runs.append(r)
    runs.sort(key=PL.ktau)
    c = colour or FLOWN.get(algo, INK)

    fig, axs = plt.subplots(1, len(runs), figsize=(FULL, height), sharey=True,
                            squeeze=False)
    # t0 drops the fan-out. The fleet starts wherever it was parked, so the
    # first few seconds are a one-off transient three times the size of
    # everything after it; left in, it owns the y-axis and flattens the part
    # of the flight the figure is about.
    lo, hi = np.inf, 0.0
    for ax, r in zip(axs[0], runs):
        if r['sim_promise'] is not None:
            st = r['sim']['t'] - r['sim']['t'][0]
            m = st >= t0
            ax.plot(st[m], r['sim_promise'][m], color=DESIGN, lw=1.8,
                    ls=(0, (4, 2)))
            lo = min(lo, float(np.nanmin(r['sim_promise'][m])))
            hi = max(hi, float(np.nanmax(r['sim_promise'][m])))
        if r['promise'] is not None:
            m = r['t'] >= t0
            ax.plot(r['t'][m], r['promise'][m], color=c, lw=1.0)
            lo = min(lo, float(np.nanmin(r['promise'][m])))
            hi = max(hi, float(np.nanmax(r['promise'][m])))
        ax.set_xlim(left=t0)
        tag(ax, f"k·τ {PL.ktau(r):.2f}")
    axs[0][0].set_ylabel(PROMISE_LABEL[spec['promise']])
    pad = (hi - lo) * 0.18
    axs[0][0].set_ylim(0 if zero else lo - pad, hi + pad * 2.2)
    axs[0][0].legend(handles=[Line2D([], [], color=c, lw=1.0, label='hardware'),
                              Line2D([], [], color=DESIGN, lw=1.8, ls=(0, (4, 2)),
                                     label='simulation, same config')],
                     loc='lower left', handlelength=1.8, borderaxespad=0.3)
    fig.supxlabel('time since the algorithm started (s)', fontsize=8,
                  color=INK)
    save(fig, stem)
    for r in runs:
        q = r['promise']
        if q is not None:
            print(f"      k.tau {PL.ktau(r):.2f}  median {np.nanmedian(q):.3f}"
                  f"  final {np.nanmean(q[-20:]):.3f}")


# ------------------------------------------------ "the law converged" figures
# Hoam's review asks for the result of each algorithm shown as a picture, not
# only as a time series: how the agents come together into a group, how they
# settle onto the Voronoi partition. These are two snapshots of the SAME quiet
# flight -- the moment the algorithm engaged and a settled moment late in it --
# with the structure the law is supposed to produce drawn over the top. The
# metric panels (promise()) say it converged; these say what it converged to.

def _voronoi(pos, half):
    """Each agent's Voronoi cell, clipped to the square region Q.

    Same construction the recorder scores H with (metrics_recorder._cells),
    reusing the controller's own half-plane clip so the drawn partition cannot
    drift out of step with the one the cost was computed on.
    """
    from drone_testbed.algorithms.coverage import _clip_halfplane
    sq = np.array([[-half, -half], [half, -half], [half, half], [-half, half]],
                  float)
    cells = []
    for k in range(len(pos)):
        poly = sq.copy()
        for j in range(len(pos)):
            if j == k:
                continue
            d = pos[j] - pos[k]
            if float(d @ d) < 1e-12:
                continue
            poly = _clip_halfplane(poly, 0.5 * (pos[k] + pos[j]), d)
            if poly is None:
                break
        cells.append(poly)
    return cells


def _snap_axes(algo, height=2.25):
    fig, axs = plt.subplots(1, 2, figsize=(COL * 2 / FULL * FULL, height),
                            squeeze=False)
    return fig, axs[0]


def converge(algo, stem, rung=None, t_late=None, height=2.3, width=COL * 1.04):
    """Two snapshots of one quiet flight: at engage, and settled."""
    import plot_ladder as PL
    import ladder_common as C
    spec = PL.LADDERS[algo]
    label, rec, cfgname = spec['runs'][0] if rung is None else \
        next(r for r in spec['runs'] if r[0] == rung)
    r = PL.analyse(label, rec, cfgname, spec)
    c = FLOWN.get(algo, INK)
    t = r['t']
    k0 = 0                      # the moment the algorithm engaged
    k1 = int(np.argmin(np.abs(t - (t_late if t_late is not None
                                   else 0.85 * t[-1]))))
    params = r['rec_obj']['params']

    fig, axs = plt.subplots(1, 2, figsize=(width * 2, height), squeeze=False)
    for ax, k in zip(axs[0], (k0, k1)):
        pos = r['P'][k]
        DRAW[algo](ax, pos, params, r, k)
        ax.plot(pos[:, 0], pos[:, 1], 'o', ms=4.5, color=c, mec='white',
                mew=0.8, zorder=5)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
        ax.grid(False)
    # The two panels must share limits or "they came together" is an artefact
    # of the right-hand panel being drawn at a different scale. Coverage sets
    # its own: there the region Q is the frame and it does not move.
    if algo != 'coverage':
        pts = np.vstack([r['P'][k0], r['P'][k1]])
        if algo == 'kuramoto':
            rad = float(params.get('radius', 0.65))
            cen = np.asarray(params.get('center', [0.0, 0.0]), float)
            pts = np.vstack([pts, cen + rad, cen - rad])
        for ax in axs[0]:
            _square_about(ax, pts, pad=0.25)
    axs[0][0].set_ylabel('y (m)')
    tag(axs[0][0], f't = {t[k0]:.0f} s')
    tag(axs[0][1], f't = {t[k1]:.0f} s')
    save(fig, stem)
    print(f"      {algo} rung {label}: snapshots at {t[k0]:.0f} s and {t[k1]:.0f} s")


def _draw_coverage(ax, pos, params, r, k):
    """Voronoi partition, each cell's centroid, and the density hotspot.

    The law's fixed points are the configurations where every agent sits on
    its own cell's centroid, so the agent-to-centroid distance IS the measure
    of convergence and both are drawn.
    """
    from drone_testbed.algorithms.coverage import Coverage
    half = float(params.get('region_half_width', 1.5))
    sq = np.array([[-half, -half], [half, -half], [half, half],
                   [-half, -half]], float)
    ax.plot(sq[:, 0], sq[:, 1], color=GRID, lw=1.0, zorder=0)
    ax.plot([half, half, -half], [-half, half, half], color=GRID, lw=1.0, zorder=0)
    algo = Coverage()
    algo.configure(dict(params), [d for d in r['rec_obj']['ids']])
    tt = r['t'][k]
    for poly in _voronoi(pos, half):
        if poly is None:
            continue
        ax.fill(poly[:, 0], poly[:, 1], facecolor='none', edgecolor=MUTED,
                lw=0.7, zorder=2)
    cen = algo._hotspot_at(tt)
    sig = float(params.get('hotspot_sigma', 0.5))
    th = np.linspace(0, 2 * np.pi, 80)
    for m, a in ((1.0, 0.16), (2.0, 0.09)):
        ax.fill(cen[0] + m * sig * np.cos(th), cen[1] + m * sig * np.sin(th),
                color='#d0761a', alpha=a, lw=0, zorder=1)
    for p, poly in zip(pos, _voronoi(pos, half)):
        if poly is None:
            continue
        cc, _ = algo._cell_integral(poly, p, tt)
        ax.plot([p[0], cc[0]], [p[1], cc[1]], color=INK, lw=0.9, zorder=4)
        ax.plot(*cc, marker='x', ms=4, color=INK, mew=1.1, zorder=4)
    ax.set_xlim(-half * 1.03, half * 1.03)
    ax.set_ylim(-half * 1.03, half * 1.03)


def _draw_flocking(ax, pos, params, r, k):
    """The lattice: every pair inside the sense range joined, length labelled.

    The edges are deliberately NOT coloured good/bad. Olfati-Saber's alpha
    lattice does not settle at the nominal `spacing` d: the sigma-norm bump
    puts the equilibrium at about 0.84 d (0.59 m for d = 0.70, as the config
    itself notes), and the recorder's lattice_err is measured against d, so a
    perfect flock still reads ~0.11 m of "error". Scoring the edges here
    against d would repeat that mistake inside the picture. The lengths are
    printed and the caption makes the comparison.
    """
    d0 = float(params.get('spacing', 0.6))
    rs = float(params.get('sense_range', 1.2 * d0))
    n = len(pos)
    for a in range(n):
        for b in range(a + 1, n):
            L = float(np.linalg.norm(pos[a] - pos[b]))
            if L > rs:
                continue
            ax.plot(*zip(pos[a], pos[b]), color=DESIGN, lw=1.6, zorder=2)
            mid = 0.5 * (pos[a] + pos[b])
            ax.text(*mid, f'{L:.2f}', fontsize=5.5, ha='center', va='center',
                    color=MUTED, zorder=3,
                    bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.6))


def _draw_kuramoto(ax, pos, params, r, k):
    """The ring the agents are supposed to end up evenly spaced on."""
    rad = float(params.get('radius', 0.65))
    cen = np.asarray(params.get('center', params.get('formation_center',
                                                     [0.0, 0.0])), float)
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(cen[0] + rad * np.cos(th), cen[1] + rad * np.sin(th),
            color=DESIGN, lw=1.8, zorder=2)
    for p in pos:
        ax.plot(*zip(cen, p), color=GRID, lw=0.7, zorder=1)


def _draw_distance(ax, pos, params, r, k):
    """The graph whose edge lengths are the entire specification."""
    from drone_testbed.algorithms.distance_formation import formation_spec
    _targets, edges, dist = formation_spec(params, r['rec_obj']['ids'])
    for (a, b), d in zip(edges, dist):
        L = float(np.linalg.norm(pos[a] - pos[b]))
        bad = min(1.0, abs(L - d) / (0.3 * d))
        ax.plot(*zip(pos[a], pos[b]),
                color=DESIGN if bad < 0.3 else '#c2571a',
                lw=1.5 if bad < 0.3 else 1.1, zorder=2)


def _square_about(ax, pts, pad=0.3):
    pts = np.asarray(pts, float).reshape(-1, 2)
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    lim = float(np.abs(pts - [cx, cy]).max()) + pad
    ax.set_xlim(cx - lim, cx + lim)
    ax.set_ylim(cy - lim, cy + lim)


DRAW = {'coverage': _draw_coverage, 'flocking': _draw_flocking,
        'kuramoto': _draw_kuramoto, 'distance': _draw_distance}


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
    promise('coverage', 'fig_coverage_promise', t0=5.0, zero=False)
    promise('flocking', 'fig_flocking_promise', t0=5.0, zero=False)
    promise('kuramoto', 'fig_kuramoto_promise', t0=5.0, zero=False)
    promise('distance', 'fig_distance_promise', t0=5.0, zero=False)
    design_rows('kuramoto', 'fig_kuramoto_design')
    benchmarks('flocking', 'fig_flocking_benchmarks')
    benchmarks('kuramoto', 'fig_kuramoto_benchmarks', native_n=4)
    for a in ('coverage', 'flocking', 'kuramoto', 'distance'):
        converge(a, f'fig_{a}_converge')


if __name__ == '__main__':
    main()
