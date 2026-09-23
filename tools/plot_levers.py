#!/usr/bin/env python3
"""The two levers that widen the gap WITHOUT changing the delay margin.

    python3 tools/plot_levers.py --lever coverage_hotspot
    python3 tools/plot_levers.py --lever flocking_sense
    python3 tools/plot_levers.py                       # both

Every ladder in tools/plot_ladder.py varies k*tau, the one number that decides
whether a flight oscillates. These two sweeps deliberately do NOT: the gains are
byte-identical across each sweep, and what changes is how hard the task is.

  coverage_hotspot  the density's orbit rate, 0.3 -> 0.6 -> 0.9 rad/s, gains
                    fixed at c2 (kp 4.0, kd 2.4), so k*tau stays at 0.67
  flocking_sense    sense_range 0.84 -> 0.78 -> 0.73 m at a fixed 0.70 m
                    spacing, s2 gains, so the graph loses edges

The point of each figure is the same: separate "the law was asked for something
harder" from "the hardware could not do what the law asked".

Two measurement notes, both load-bearing:

* COVERAGE COST IS SCORED AT EACH RUN'S OWN BEST PHASE. The recorder's H column
  is scored against the hotspot on the recorder's clock (PROJECT_AIM 15d), and
  even after recomputing on the algorithm's clock, t_start is only recoverable
  to about a tick -- the virtual agents are detected on their first MOVE, which
  is one command after the algorithm actually started. At 0.9 rad/s a 0.25 s
  ambiguity is 13 degrees of hotspot phase and it dominates the comparison. So
  the scan below reports two separate numbers instead of one confounded one:
  the phase the fleet actually ran at, and the cost it achieved there.

* SETTLED AVERAGES RUN OVER WHOLE REFERENCE ORBITS. Both algorithms chase a
  moving reference whose period is comparable to the flight, so a "last 20%"
  average lands on a different part of the orbit in every run and reports the
  difference as a result. Each average here covers a whole number of orbits.
"""

import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                   # noqa: E402

import ladder_common as C                                          # noqa: E402

plt.rcParams.update(C.RC)

COV, FLK = '#eb6834', '#1baf7a'          # the palette plot_key.py validated
SETTLE = 15.0                            # s of the window kept for the transient

LEVERS = {
    'coverage_hotspot': dict(
        algo='Coverage', duration=90.0, promise='H', colour=COV,
        lever='hotspot_speed', unit='rad/s', axis='hotspot orbit rate (rad/s)',
        title='Coverage: the same law, a faster target',
        # 22 Sep only. The 16 Sep c2 flight runs the same config and is kept in
        # the k*tau ladder, but it is a different session -- different battery
        # set, different VICON calibration, drones on different marks -- and
        # mixing sessions inside one sweep puts a session difference on the
        # same axis as the lever. The sweep is one afternoon or it is nothing.
        runs=[('0.3', 'coverage_20260922_151551.txt', 'testbed_coverage_c2.yaml'),
              ('0.6', 'coverage_20260922_152013.txt', 'testbed_coverage_c2_h06.yaml'),
              ('0.9', 'coverage_20260922_152611.txt', 'testbed_coverage_c2_h09.yaml')],
    ),
    'flocking_sense': dict(
        algo='Flocking', duration=150.0, promise='lattice_err', colour=FLK,
        lever='sense_range', unit='m', axis='sense range (m), spacing fixed at 0.70',
        title='Flocking: the same law, fewer neighbours',
        # 22 Sep only, for the reason above -- so this sweep is two points.
        runs=[('0.78', 'flocking_20260922_153713.txt', 'testbed_flocking_hybrid_s2_r78.yaml'),
              ('0.73', 'flocking_20260922_154214.txt', 'testbed_flocking_hybrid_s2_r73.yaml')],
    ),
}


# ---- measurement -----------------------------------------------------------

def reference_period(params, algo):
    """Seconds per lap of the thing the fleet is chasing."""
    w = float(params.get('hotspot_speed' if algo == 'Coverage' else 'gamma_speed', 0.0))
    return 2 * np.pi / w if w > 0 else None


def orbit_mean(y, t, period):
    """Mean over the last WHOLE reference orbits that fit after the transient."""
    y = np.asarray(y, dtype=float)
    if period is None or not np.isfinite(period) or period <= 0:
        return float(np.nanmean(y[int(0.8 * len(y)):]))
    n = int((t[-1] - t[0] - SETTLE) // period)
    if n < 1:
        return float(np.nanmean(y[int(0.8 * len(y)):]))
    return float(np.nanmean(y[np.searchsorted(t, t[-1] - n * period):]))


def phase_scan(ids, params, P, t_rel, offs, stride=3):
    """Coverage cost as a function of how the hotspot's clock is shifted.

    The minimum is where the fleet actually is: a positive lag means the fleet
    is scored best against where the hotspot WAS, i.e. it is running behind.
    """
    import metrics_recorder as M
    cm = M.CoverageMetrics(ids, params)
    ks = range(0, len(t_rel), stride)
    return np.array([np.mean([sum(c[1] for c in cm._cells(P[k], t_rel[k] + o)
                                  if c is not None) for k in ks]) for o in offs])


def analyse(label, rec_name, cfgname, spec, offs):
    path = os.path.join(C.LOGS, rec_name)
    if not os.path.exists(path):
        print(f'  [skip] {label}: {rec_name} not in logs/hw/')
        return None
    rec = C.load(path)
    P = C.positions(rec)
    if P is None:
        print(f'  [skip] {label}: {rec_name} has no positions')
        return None
    w = C.window(rec, P)
    sl = w['sl']
    Pw = P[sl]
    t = rec['t'][sl] - rec['t'][sl][0]
    period = reference_period(rec['params'], spec['algo'])
    clamp = float(rec['params'].get('max_accel', 0.5))

    U, Uc = C.replay(spec['algo'], rec['params'], rec['ids'], Pw, w['t_start'], len(Pw))
    Acc = np.gradient(np.gradient(Pw, C.DT, axis=0), C.DT, axis=0)
    V = np.gradient(Pw, C.DT, axis=0)
    wb = C.wobble(Pw)
    ks = C.own_velocity_gain(spec['algo'], rec['params'], P, sl)
    j = w['real'][0]

    sim = C.sim_of(cfgname, spec['duration'])
    out = dict(
        label=label, rec=rec_name, cfg=cfgname, window=float(t[-1]),
        lever=float(rec['params'][spec['lever']]), period=period,
        k=float(ks[j]), k_tau=float(ks[j] * C.TAU_NOMINAL),
        wobble=float(wb['rms'][j]), own=float(wb['own_rms'][j]),
        shared=float(wb['shared_rms']),
        sim_wobble=[float(wb['rms'][m]) for m in w['virt']],
        clip=float(np.mean(np.any(np.abs(U[:, j]) > clamp - 1e-9, axis=1))),
        tilt_med=float(np.median(C.tilt(Acc[:, j]))),
        speed_med=float(np.median(np.hypot(*V[:, j].T))),
        speed_max=float(np.max(np.hypot(*V[:, j].T))),
        tau=float(C.lag(Uc[:, j], Acc[:, j])[2]),
        P=Pw, t=t, sim=sim, j=j,
    )

    if spec['algo'] == 'Coverage':
        t_rel = rec['t'][sl] - w['t_start']
        Hh = phase_scan(rec['ids'], rec['params'], Pw, t_rel, offs)
        ms = sim['t'] > SETTLE
        Hs = phase_scan(sim['ids'], sim['cfg']['algorithm']['params'],
                        np.array(sim['P'])[ms], sim['t'][ms], offs)
        out.update(scan_hw=Hh.tolist(), scan_sim=Hs.tolist(),
                   lag_hw=float(-offs[Hh.argmin()]), lag_sim=float(-offs[Hs.argmin()]),
                   promise=float(Hh.min()), promise_sim=float(Hs.min()))
        out['lag_deg'] = out['lag_hw'] / period * 360.0
        out['lag_deg_sim'] = out['lag_sim'] / period * 360.0
    else:
        p, ps = C.col(rec, spec['promise']), C.col(sim, spec['promise'])
        out.update(promise=orbit_mean(p[sl], t, period),
                   promise_sim=orbit_mean(ps, sim['t'], period))

    for extra in ('connected', 'd_min', 'vel_spread', 'cdist_max'):
        h, s = C.col(rec, extra), C.col(sim, extra)
        if h is not None:
            out[extra] = orbit_mean(h[sl], t, period)
        if s is not None:
            out[extra + '_sim'] = orbit_mean(s, sim['t'], period)
    if spec['algo'] == 'Flocking':
        # The four shortest of the six pairwise distances are the neighbour
        # pairs (the other two are the diagonals of the diamond). That mean is
        # the spacing the flock actually settled on, against the `spacing`
        # parameter the law was told to hold.
        for tag, src, idx in (('', rec, sl), ('_sim', sim, slice(None))):
            d = np.sort(np.stack([C.col(src, c)[idx] for c in src['cols']
                                  if c.startswith('d_drone')], 1), axis=1)
            out['side' + tag] = float(np.mean(d[-300:, :4]))
            out['diag' + tag] = float(np.mean(d[-300:, 4:]))
        out['spacing'] = float(rec['params']['spacing'])
    return out


# ---- figures ---------------------------------------------------------------

def _x(runs):
    return [r['lever'] for r in runs]


def _pair_bars(ax, runs, key, colour, label_hw, scale=1.0, fmt='{:.3f}'):
    """Hardware against simulation of the same file, one pair per rung."""
    xs = np.arange(len(runs))
    hw = [r[key] * scale for r in runs]
    sm = [r.get(key + '_sim', np.nan) * scale for r in runs]
    ax.bar(xs - 0.20, hw, 0.38, color=colour, label=label_hw)
    ax.bar(xs + 0.20, sm, 0.38, color=C.SIM, label='simulation')
    for k, (h, s) in enumerate(zip(hw, sm)):
        if np.isfinite(s) and s != 0:
            ax.annotate(f'{100 * (h / s - 1):+.0f}%', (k, max(h, s)),
                        textcoords='offset points', xytext=(0, 3),
                        ha='center', fontsize=7.5, color=C.MUTED)
    ax.set_xticks(xs)
    return hw, sm


def fig_coverage(runs, spec, out):
    fig, ax = plt.subplots(1, 3, figsize=(13.4, 4.6))
    fig.subplots_adjust(wspace=0.30)
    cc = spec['colour']
    xs = np.arange(len(runs))
    names = [f"{r['label']} rad/s" for r in runs]

    # 1 -- the lever leaves the delay margin alone
    a = ax[0]
    a.bar(xs, [r['wobble'] * 100 for r in runs], 0.5, color=cc, zorder=2)
    for k, r in enumerate(runs):
        a.annotate(f"{r['wobble'] * 100:.2f} cm", (k, r['wobble'] * 100),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.INK)
    top = 1.35 * max(r['wobble'] * 100 for r in runs)
    a.set_xticks(xs)
    a.set_xticklabels([f"{n}\nk·τ {r['k_tau']:.2f}" for n, r in zip(names, runs)])
    a.set_ylim(0, top)
    a.set_ylabel('oscillation radius, real drone (cm)')
    a.set_title('Oscillation radius')

    # 2 -- the fleet runs a fixed TIME behind, which is a growing ANGLE
    a = ax[1]
    a.bar(xs - 0.20, [r['lag_deg'] for r in runs], 0.38, color=cc,
          label='hardware', zorder=2)
    a.bar(xs + 0.20, [r['lag_deg_sim'] for r in runs], 0.38, color=C.SIM,
          label='simulation', zorder=2)
    for k, r in enumerate(runs):
        a.annotate(f"{r['lag_hw']:.2f} s", (k - 0.20, r['lag_deg']),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.INK)
        a.annotate(f"{r['lag_sim']:.2f} s", (k + 0.20, r['lag_deg_sim']),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.MUTED)
    a.set_xticks(xs)
    a.set_xticklabels(names)
    a.set_ylim(0, 1.25 * max(r['lag_deg_sim'] for r in runs))
    a.set_ylabel('lag behind the hotspot (° of orbit)')
    a.set_title('How far behind the hotspot the fleet flies')

    # 3 -- and at that phase, the achieved cost is simulation's
    a = ax[2]
    _pair_bars(a, runs, 'promise', cc, 'hardware')
    a.set_xticklabels(names)
    a.set_ylabel('coverage cost H (lower = better covered)')
    a.set_title('Coverage cost')

    fig.legend(*ax[1].get_legend_handles_labels(), fontsize=8.5, ncol=2,
               loc='lower center', bbox_to_anchor=(0.5, -0.07), frameon=False)
    fig.suptitle(f"Coverage, hotspot {' / '.join(r['label'] for r in runs)} rad/s "
                 f"— gains unchanged, so k·τ is {runs[0]['k_tau']:.2f} in all three",
                 y=1.02, fontsize=12, fontweight='bold')
    fig.savefig(os.path.join(out, '1_hotspot_speed.png'))
    plt.close(fig)


def fig_flocking(runs, spec, out):
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 4.6))
    fig.subplots_adjust(wspace=0.32)
    cc = spec['colour']
    xs = np.arange(len(runs))
    names = [f"{r['label']} m" for r in runs]

    # 1 -- the spacing the flock settled on, against the spacing it was told to
    # hold. This is what "lattice error" is, in metres, without the word.
    a = ax[0]
    a.bar(xs - 0.20, [r['side'] for r in runs], 0.38, color=cc,
          label='hardware', zorder=2)
    a.bar(xs + 0.20, [r['side_sim'] for r in runs], 0.38, color=C.SIM,
          label='simulation', zorder=2)
    target = runs[0]['spacing']
    a.axhline(target, color=C.INK, ls='--', lw=1.4, zorder=3)
    a.annotate(f'the law asks for {target:.2f} m', (len(runs) - 0.55, target),
               xytext=(0, 4), textcoords='offset points', ha='right',
               fontsize=8, color=C.INK)
    for k, r in enumerate(runs):
        a.annotate(f"{r['side']:.3f}", (k - 0.20, r['side']),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.INK)
        a.annotate(f"{r['side_sim']:.3f}", (k + 0.20, r['side_sim']),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.MUTED)
    a.set_xticks(xs)
    a.set_xticklabels(names)
    a.set_ylim(0, 0.85)
    a.set_ylabel('distance between neighbouring drones (m)')
    a.set_title('Settled spacing between neighbours')

    # 2 -- the ripple
    a = ax[1]
    a.bar(xs, [r['wobble'] * 100 for r in runs], 0.45, color=cc, zorder=2)
    for k, r in enumerate(runs):
        a.annotate(f"{r['wobble'] * 100:.2f} cm", (k, r['wobble'] * 100),
                   textcoords='offset points', xytext=(0, 3), ha='center',
                   fontsize=7.5, color=C.INK)
    a.set_xticks(xs)
    a.set_xticklabels([f"{n}\nk·τ {r['k_tau']:.2f}" for n, r in zip(names, runs)])
    a.set_ylim(0, 1.0)
    a.set_ylabel('oscillation radius, real drone (cm)')
    a.set_title('Oscillation radius')

    fig.legend(*ax[0].get_legend_handles_labels(), fontsize=8.5, ncol=2,
               loc='lower center', bbox_to_anchor=(0.5, -0.07), frameon=False)
    fig.suptitle(f"Flocking, sense range {' / '.join(r['label'] for r in runs)} m "
                 f"— gains and spacing unchanged", y=1.02,
                 fontsize=12, fontweight='bold')
    fig.savefig(os.path.join(out, '1_sense_range.png'))
    plt.close(fig)


def topology_sweep(cfgname, rs, secs=150.0, cache=None):
    """Settled pairwise distances and edge count against sense range.

    Which edges the graph has is decided by where r falls between the flock's
    settled SIDE (~0.57 m) and its settled DIAGONAL (~0.81 m), not by r on its
    own -- so two different r values inside the same band are the same graph.
    """
    import copy
    import yaml
    import sim_baseline as S
    if cache and os.path.exists(cache):
        with open(cache) as fh:
            return json.load(fh)
    base = yaml.safe_load(open(os.path.join(C.CFG, cfgname)))
    out = []
    for r in rs:
        cfg = copy.deepcopy(base)
        cfg['algorithm']['params']['sense_range'] = float(r)
        metrics, ts, D, ph, _ = S.run(cfg, secs)
        cols = metrics.all_columns()
        m = ts > 0.7 * secs
        d = np.stack([D[:, cols.index(c)] for c in cols
                      if c.startswith('d_drone')], 1)[m]
        pairs = sorted(float(v) for v in d.mean(0))
        out.append(dict(r=float(r), pairs=pairs,
                        edges=int(round(float((d < r).sum(1).mean()))),
                        lattice=float(D[m, cols.index('lattice_err')].mean())))
    if cache:
        with open(cache, 'w') as fh:
            json.dump(out, fh, indent=1)
    return out


def fig_topology(runs, spec, out, flown=(0.78, 0.73), planned=(0.78, 0.86, 1.05)):
    """Why the 22 Sep sense-range sweep found nothing."""
    sw = topology_sweep('testbed_flocking_hybrid_s2_r78.yaml',
                        np.round(np.arange(0.72, 1.11, 0.03), 2),
                        cache=os.path.join(out, 'topology_sweep.json'))
    rs = np.array([x['r'] for x in sw])
    P = np.array([x['pairs'] for x in sw])          # 6 sorted pair distances
    edges = np.array([x['edges'] for x in sw])
    latt = np.array([x['lattice'] for x in sw])
    cc = spec['colour']

    fig, ax = plt.subplots(1, 2, figsize=(11.6, 4.5))
    fig.subplots_adjust(wspace=0.30)
    BANDS = ((4, 0.70, 0.805, '4 edges\nthe bare cycle'),
             (5, 0.805, 0.955, '5 edges\n+ one diagonal'),
             (6, 0.955, 1.12, '6 edges\ncomplete'))

    # A -- the graph only has three possible shapes, in bands of r
    a = ax[0]
    a.step(rs, edges, where='mid', color=cc, lw=2.2)
    for n, lo, hi, lab in BANDS:
        a.axvspan(lo, hi, color=C.GRID, alpha=0.5 if n % 2 else 0.22, zorder=0)
        a.annotate(lab, ((lo + hi) / 2, 6.55), ha='center', fontsize=8,
                   color=C.MUTED)
    for r in flown:
        a.plot([r], [4], 'v', ms=11, color=C.INK, zorder=4)
    a.annotate('both rungs flown 22 Sep', (np.mean(flown), 4),
               xytext=(0, -30), textcoords='offset points', ha='center',
               fontsize=8.5, color=C.INK)
    a.set_xlim(0.70, 1.12)
    a.set_ylim(3.3, 7.0)
    a.set_yticks([4, 5, 6])
    a.set_xlabel('sense range (m)')
    a.set_ylabel('edges in the neighbour graph (of 6)')
    a.set_title('0.78 and 0.73 are the same graph')

    # B -- and the effect available inside one band is small next to the effect
    # available between bands
    a = ax[1]
    for n, lo, hi, _ in BANDS:
        a.axvspan(lo, hi, color=C.GRID, alpha=0.5 if n % 2 else 0.22, zorder=0)
    a.plot(rs, latt, color=cc, lw=2.0, zorder=2)
    for r in flown:
        a.plot([r], [np.interp(r, rs, latt)], 'v', ms=11, color=C.INK, zorder=4)
    for r in planned:
        a.plot([r], [np.interp(r, rs, latt)], 'o', ms=10, mfc='white', mew=2.2,
               color=cc, zorder=4)
    a.plot([], [], 'v', ms=9, color=C.INK, label='flown 22 Sep')
    a.plot([], [], 'o', ms=9, mfc='white', mew=2.0, color=cc,
           label='proposed: one per band')
    a.legend(fontsize=8, loc='upper right')
    a.annotate('0.78 is kept —\nit is the 4-edge rung',
               (planned[0], np.interp(planned[0], rs, latt)),
               xytext=(12, 16), textcoords='offset points', ha='left',
               fontsize=7.5, color=C.MUTED)
    a.set_xlim(0.70, 1.12)
    a.set_xlabel('sense range (m)')
    a.set_ylabel('lattice error in simulation (m)')
    a.set_title('More edges is not simply better')

    fig.suptitle('Why the sense-range sweep found nothing', y=1.02,
                 fontsize=12, fontweight='bold')
    fig.savefig(os.path.join(out, '0_why_null.png'))
    plt.close(fig)
    return sw


def rung_name(r, unit, sep=' · '):
    """'0.3 rad/s · 16 Sep' -- the unit belongs to the value, not to the date."""
    parts = r['label'].split('\n')
    return f'{parts[0]} {unit}' + (sep + parts[1] if len(parts) > 1 else '')


def deviation(r, span=None):
    """Distance from where the law wanted the real drone, at every row.

    "Where the law wanted it" is simulation of the SAME config -- the physical
    layer removed and nothing else changed -- so this is gap B directly, in
    metres, once per 0.1 s row. The +2.0 s is the settle the window already
    dropped off the front of the hardware trace.
    """
    st = r['sim']['t'] - r['sim']['t'][0]
    m = np.ones(len(r['t']), bool) if span is None else (
        (r['t'] >= span[0]) & (r['t'] <= span[1]))
    want = np.stack([np.interp(r['t'][m] + 2.0, st, r['sim']['P'][:, r['j'], ax])
                     for ax in (0, 1)], 1)
    return r['t'][m], r['P'][m, r['j']], want, np.hypot(*(r['P'][m, r['j']] - want).T)


def fig_deviation(runs, spec, out, span=25.0):
    """Design against actual, and the whole distribution of the gap per rung."""
    cc = spec['colour']
    n = len(runs)
    fig = plt.figure(figsize=(3.4 * n, 7.0))
    gs = fig.add_gridspec(2, n, height_ratios=[1.1, 1], hspace=0.28)

    for k, r in enumerate(runs):
        a = fig.add_subplot(gs[0, k])
        mid = 0.5 * r['t'][-1]
        _, got, want, gap = deviation(r, (mid - span / 2, mid + span / 2))
        a.plot(want[:, 0], want[:, 1], color=C.SIM, ls='--', lw=1.8,
               label='where the law wanted it')
        a.plot(got[:, 0], got[:, 1], color=cc, lw=2.0, label='where it went')
        a.set_aspect('equal')
        a.set_title(f"{rung_name(r, spec['unit'], chr(10))}\n"
                    f"{span:.0f} s mid-flight, {np.median(gap) * 100:.0f} cm off here",
                    fontsize=9.5)
        a.set_xlabel('x (m)')
        if k == 0:
            a.set_ylabel('y (m)')
            a.legend(fontsize=7.5, loc='upper left')

    # the distribution, not just the median: one box per rung over the WHOLE
    # settled window, so a rung that is usually fine and occasionally terrible
    # cannot hide behind its median
    b = fig.add_subplot(gs[1, :])
    data = [deviation(r)[3] * 100 for r in runs]
    bp = b.boxplot(data, whis=(5, 95), widths=0.55, patch_artist=True,
                   showfliers=False, medianprops=dict(color=C.INK, lw=1.8))
    for patch in bp['boxes']:
        patch.set_facecolor(cc)
        patch.set_edgecolor(C.INK)
        patch.set_alpha(0.85)
    for part in ('whiskers', 'caps'):
        for line in bp[part]:
            line.set_color(C.MUTED)
    for k, d in enumerate(data, start=1):
        b.annotate(f'median {np.median(d):.0f} cm', (k + 0.31, np.median(d)),
                   textcoords='offset points', xytext=(3, -3), ha='left',
                   fontsize=8, color=C.INK)
    b.set_xticklabels([rung_name(r, spec['unit'], chr(10)) for r in runs])
    b.set_ylabel('distance from where\nthe law wanted it (cm)')
    b.set_ylim(0, 1.12 * max(np.percentile(d, 95) for d in data))
    b.set_title('Distribution of that distance over the whole flight  '
                '(box = quartiles, whiskers = 5th–95th percentile)', fontsize=10)

    fig.suptitle(f"{spec['algo']}: how far the drone was from where the law "
                 f"wanted it", y=0.98, fontsize=12, fontweight='bold')
    fig.savefig(os.path.join(out, '3_design_vs_actual.png'))
    plt.close(fig)


# ---- table -----------------------------------------------------------------

def table(runs, spec):
    print(f"\n{spec['title']}")
    head = (f"{spec['lever']:>14} {'k':>5} {'k·τ':>5} {'τ':>6} {'ripple':>8} "
            f"{'clip':>5} {'tilt':>6} {'v med':>6} {'window':>7}")
    print(head)
    print('-' * len(head))
    for r in runs:
        print(f"{r['label']:>14} {r['k']:5.2f} {r['k_tau']:5.2f} {r['tau']:5.2f}s "
              f"{r['wobble'] * 100:7.2f}cm {r['clip'] * 100:4.0f}% "
              f"{r['tilt_med']:5.1f}° {r['speed_med']:5.2f} {r['window']:6.0f}s")
        if 'lag_hw' in r:
            print(f"{'':>14}   H {r['promise']:.4f} vs sim {r['promise_sim']:.4f} "
                  f"({100 * (r['promise'] / r['promise_sim'] - 1):+.1f}%), "
                  f"fleet lag {r['lag_hw']:.2f} s = {r['lag_deg']:.0f}° "
                  f"(sim {r['lag_sim']:.2f} s = {r['lag_deg_sim']:.0f}°)")
        else:
            print(f"{'':>14}   lattice {r['promise']:.4f} vs sim {r['promise_sim']:.4f} "
                  f"({100 * (r['promise'] / r['promise_sim'] - 1):+.1f}%), "
                  f"connected {100 * r['connected']:.1f}%, side {r['side']:.3f} m, "
                  f"d_min {r['d_min']:.3f} m")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--lever', choices=sorted(LEVERS), default=None)
    ap.add_argument('--out', default=os.path.join(C.ROOT, 'docs', 'figures'))
    a = ap.parse_args()

    offs = np.arange(-3.0, 1.001, 0.05)
    for key in ([a.lever] if a.lever else sorted(LEVERS)):
        spec = LEVERS[key]
        out = os.path.join(a.out, key)
        os.makedirs(out, exist_ok=True)
        runs = [x for x in (analyse(l, r, c, spec, offs) for l, r, c in spec['runs'])
                if x is not None]
        if not runs:
            print(f'{key}: no records yet')
            continue
        table(runs, spec)
        if spec['algo'] == 'Coverage':
            fig_coverage(runs, spec, out)
        else:
            fig_topology(runs, spec, out)
            fig_flocking(runs, spec, out)
        fig_deviation(runs, spec, out)
        with open(os.path.join(out, 'summary.json'), 'w') as fh:
            drop = ('scan_hw', 'scan_sim', 'P', 't', 'sim')
            json.dump({r['label']: {k: v for k, v in r.items() if k not in drop}
                       for r in runs}, fh, indent=1)
        print(f'\nfigures -> {out}')


if __name__ == '__main__':
    main()
