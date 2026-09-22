#!/usr/bin/env python3
"""Figures for the coverage and flocking speed ladders.

    python3 tools/plot_ladder.py --algo coverage
    python3 tools/plot_ladder.py --algo flocking
    python3 tools/plot_ladder.py --algo flocking --runs "k1.3=flocking_x.txt:testbed_flocking_hybrid.yaml"

Both ladders are a time rescale: the law is unchanged and the trajectory is
unchanged, only the clock -- so the own-velocity gain k falls and `k * tau`
crosses the ~1 threshold from docs/PROJECT_AIM.md section 13. Every number here
is computed from the raw rows over the clean window by tools/ladder_common.py,
which is the same method the trochoidal ladder used.

A missing record is skipped with a warning, so this runs before every rung has
been flown.
"""

import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402

import ladder_common as C                                         # noqa: E402

plt.rcParams.update(C.RC)

# (rung label, record, config, k*tau as designed) -- one real drone, drone1
LADDERS = {
    'coverage': dict(
        algo='Coverage', duration=90.0, promise='H',
        runs=[('c1', 'coverage_20260916_161129.txt', 'testbed_coverage_c1.yaml'),
              ('c2', 'coverage_20260916_161409.txt', 'testbed_coverage_c2.yaml'),
              ('c3', 'coverage_20260916_161948.txt', 'testbed_coverage_c3.yaml')],
    ),
    'flocking': dict(
        algo='Flocking', duration=150.0, promise='lattice_err',
        runs=[('s3', 'flocking_20260916_165909.txt', 'testbed_flocking_hybrid_s3.yaml'),
              ('s2', 'flocking_20260916_165223.txt', 'testbed_flocking_hybrid_s2.yaml'),
              # s1 is the UNRESCALED law (c = 1), not a reference run: the
              # reference every rung is scored against is simulation of its own
              # config. s2 = half speed, s3 = a third.
              ('s1', 'flocking_20260916_164800.txt', 'testbed_flocking_hybrid.yaml')],
    ),
}


def analyse(label, rec_name, cfgname, spec):
    path = os.path.join(C.LOGS, rec_name)
    if not os.path.exists(path):
        print(f'  [skip] {label}: {rec_name} not in logs/hw/')
        return None
    rec = C.load(path)
    P = C.positions(rec)
    if P is None:
        print(f'  [skip] {label}: {rec_name} predates position logging')
        return None
    w = C.window(rec, P)
    sl = w['sl']
    Pw, tw = P[sl], rec['t'][sl]
    clamp = float(rec['params'].get('max_accel', 0.5))

    U, Uc = C.replay(spec['algo'], rec['params'], rec['ids'], Pw, w['t_start'], len(Pw))
    Acc = np.gradient(np.gradient(Pw, C.DT, axis=0), C.DT, axis=0)
    V = np.gradient(Pw, C.DT, axis=0)
    wb = C.wobble(Pw)
    ks = C.own_velocity_gain(spec['algo'], rec['params'], P, sl)

    drones = {}
    for j, i in enumerate(rec['ids']):
        lags, xc, L, xcmax = C.lag(Uc[:, j], Acc[:, j])
        drones[i] = dict(
            real=j in w['real'], j=j, k=ks[j], k_tau=ks[j] * C.TAU_NOMINAL,
            lags=lags, xc=xc, lag=L, xc_max=xcmax,
            wobble=wb['rms'][j], own=wb['own_rms'][j], period=wb['period'][j],
            circ=wb['circ'][j], sense=wb['sense'][j],
            clip=float(np.mean(np.any(np.abs(U[:, j]) > clamp - 1e-9, axis=1))),
            u_med=float(np.median(np.hypot(*U[:, j].T))),
            a_med=float(np.median(np.hypot(*Acc[:, j].T))),
            tilt_med=float(np.median(C.tilt(Acc[:, j]))),
            speed_med=float(np.median(np.hypot(*V[:, j].T))),
            speed_max=float(np.max(np.hypot(*V[:, j].T))),
        )

    sim = C.sim_of(cfgname, spec['duration'])
    promise = C.col(rec, spec['promise'])
    promise_logged = None
    if spec['algo'] == 'Coverage' and promise is not None:
        # The recorder evaluates the coverage cost against the moving hotspot at
        # ITS OWN t, which starts when the recorder starts. The algorithm's
        # hotspot clock starts when the algorithm does, ~12.5 s later at
        # takeoff. With hotspot_speed 0.3 rad/s that is ~3.8 rad of phase, so
        # the logged H scores the drones against a hotspot on the far side of
        # the orbit. Recompute it on the algorithm's clock; keep the logged
        # series so the figure can show the difference.
        promise_logged = promise.copy()
        promise = C.recompute_coverage_H(rec, P, w['t_start'])
    sim_promise = C.col(sim, spec['promise'])
    tail = slice(int(0.8 * len(Pw)), None)
    out = dict(label=label, rec=rec_name, cfg=cfgname, rec_obj=rec, sim=sim,
               P=Pw, t=tw - tw[0], U=U, Uc=Uc, Acc=Acc, clamp=clamp,
               drones=drones, shared=wb['shared_rms'], window=float(tw[-1] - tw[0]),
               promise=None if promise is None else promise[sl],
               promise_logged=None if promise_logged is None else promise_logged[sl],
               sim_promise=sim_promise, real=w['real'], virt=w['virt'])
    if promise is not None:
        pw = promise[sl]
        out['promise_settled'] = float(np.mean(pw[tail]))
        out['promise_sim_settled'] = (float(np.mean(sim_promise[int(0.8 * len(sim_promise)):]))
                                      if sim_promise is not None else float('nan'))
    for extra in ('cdist_max', 'd_min', 'vel_spread', 'connected'):
        v = C.col(rec, extra)
        if v is not None:
            out[extra] = float(np.mean(v[sl][tail]))
    return out


# ---- figures ---------------------------------------------------------------

def ktau(r):
    return float(np.mean([d['k_tau'] for d in r['drones'].values()]))


def rung_label(r, two_line=True):
    """k·τ first, because that is the axis everything is ordered on; the rung
    name second, because that is what the config and the record are called."""
    sep = '\n' if two_line else '  '
    return f"k·τ {ktau(r):.2f}{sep}({r['label']})"


def _cols(runs, algo):
    ramp = C.RAMP[algo]
    return {r['label']: ramp[min(k, len(ramp) - 1)] for k, r in enumerate(runs)}


def fig_summary(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.4))
    x = np.arange(len(runs))

    # 1: wobble, real vs virtual, against the designed k*tau
    for k, r in enumerate(runs):
        for i, d in r['drones'].items():
            ax[0].plot(k, d['wobble'] * 100, C.MARKERS[d['j'] % 4],
                       ms=9 if d['real'] else 6,
                       mfc=cc[r['label']] if d['real'] else 'white',
                       mec=cc[r['label']], mew=1.6, zorder=3)
    ax[0].set_xticks(x, [rung_label(r) for r in runs])
    allw = [d['wobble'] * 100 for r in runs for d in r['drones'].values() if d['wobble'] > 0]
    if allw and max(allw) / min(allw) > 20:
        ax[0].set_yscale('log')
        ax[0].set_ylabel('radius of oscillation (cm, log scale)')
    else:
        ax[0].set_ylabel('radius of oscillation (cm)')
    ax[0].set_title('Oscillation size vs the delay margin')
    ax[0].plot([], [], 'o', color=C.INK, label='real drone (filled)')
    ax[0].plot([], [], 'o', mfc='white', mec=C.INK, label='simulated drone')
    ax[0].legend(fontsize=7.5, loc='upper left')

    # 2: the promised property, hardware vs its own simulation
    lab = {'H': 'coverage cost H (settled)',
           'lattice_err': 'lattice error (settled, m)'}[spec['promise']]
    for k, r in enumerate(runs):
        if 'promise_settled' not in r:
            continue
        ax[1].bar(k - 0.19, r['promise_settled'], 0.36, color=cc[r['label']],
                  label='hardware' if k == 0 else None)
        ax[1].bar(k + 0.19, r['promise_sim_settled'], 0.36, color=C.SIM,
                  label='simulation' if k == 0 else None)
        gap = r['promise_settled'] / r['promise_sim_settled'] - 1
        ax[1].annotate(f'{gap*100:+.0f}%', (k, max(r['promise_settled'],
                       r['promise_sim_settled'])), ha='center', va='bottom',
                       fontsize=7.5, color=C.MUTED)
    ax[1].set_xticks(x, [rung_label(r) for r in runs])
    ax[1].set_ylabel(lab)
    ax[1].set_title('Does the promise survive?')
    ax[1].legend(fontsize=7.5)

    # 3: measured delay and whether the ripple sits at 4*tau
    for k, r in enumerate(runs):
        for i, d in r['drones'].items():
            if not d['real']:
                continue
            weak = d['xc_max'] < 0.5
            ax[2].plot(4 * d['lag'], d['period'], C.MARKERS[d['j'] % 4], ms=9,
                       mfc='white' if weak else cc[r['label']],
                       mec=cc[r['label']], mew=1.6, zorder=3,
                       label=r['label'] if i == list(r['drones'])[0] else None)
            if weak:
                ax[2].annotate('no wobble\nto time', (4 * d['lag'], d['period']),
                               textcoords='offset points', xytext=(-6, -24),
                               ha='center', fontsize=6.5, color=C.MUTED)
    lim = [0.4, 1.8]
    ax[2].plot(lim, lim, '--', color=C.DESIGN, lw=1, zorder=1)
    ax[2].annotate('period = 4τ', (1.55, 1.62), color=C.MUTED, fontsize=7.5,
                   rotation=38)
    ax[2].set_xlim(lim); ax[2].set_ylim(lim)
    ax[2].set_xlabel('4 × measured loop delay (s)')
    ax[2].set_ylabel('measured ripple period (s)')
    ax[2].set_title('The ripple is delay-timed')
    ax[2].legend(fontsize=7.5)

    fig.suptitle(f'{spec["algo"]} speed ladder — one real drone, '
                 f'{len(runs)} rungs', y=1.04, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '1_summary.png'))
    plt.close(fig)


def fig_expected_vs_actual(runs, algo, spec, out, span=25.0):
    """Design against actual: where the law wanted the drone, and where it went.

    "The law wanted" is the same config in simulation -- the physical layer
    removed and nothing else changed -- so the distance between the two lines is
    gap B, the thing this project exists to explain.

    The setpoint crazyflie_node streamed is deliberately NOT drawn. For a law
    chasing a moving reference the integrated setpoint sits on the node's 0.3 m
    leash almost continuously (94-97% of ticks here, against 6-8% for
    trochoidal), so the curve carries the leash's shape rather than the law's.
    C.setpoint() still reconstructs it for anyone who wants to look.
    """
    cc = _cols(runs, algo)
    fig = plt.figure(figsize=(3.7 * len(runs), 6.2))
    gs = fig.add_gridspec(2, len(runs), height_ratios=[2.0, 1], hspace=0.42)
    lo = fig.add_subplot(gs[1, :])

    for k, r in enumerate(runs):
        a = fig.add_subplot(gs[0, k])
        j = r['real'][0] if r['real'] else 0
        mid = 0.5 * r['t'][-1]
        t0, t1 = max(r['t'][0], mid - span / 2), min(r['t'][-1], mid + span / 2)
        m = (r['t'] >= t0) & (r['t'] <= t1)

        # the hardware window starts 2 s after the algorithm did; sim starts at 0
        st = r['sim']['t'] - r['sim']['t'][0]
        want = np.stack([np.interp(r['t'][m] + 2.0, st, r['sim']['P'][:, j, ax])
                         for ax in (0, 1)], 1)
        a.plot(want[:, 0], want[:, 1], color=C.SIM, ls='--', lw=1.8,
               label='where the law wanted it')
        a.plot(r['P'][m, j, 0], r['P'][m, j, 1], color=cc[r['label']], lw=2.0,
               label='where it actually went')
        gap = np.hypot(*(r['P'][m, j] - want).T) * 100
        a.set_aspect('equal')
        a.set_title(f"{rung_label(r, two_line=False)}"
                    f"\nmedian {np.median(gap):.0f} cm off", fontsize=9.5)
        a.set_xlabel('x (m)')
        if k == 0:
            a.set_ylabel('y (m)')
            a.legend(fontsize=7.5, loc='upper left')
        # Each rung's window is the middle of ITS OWN flight, and the flights
        # are not the same length, so the traces are laid over a common
        # "seconds into the window" axis rather than wall clock -- otherwise
        # they sit side by side and cannot be compared.
        lo.plot(r['t'][m] - r['t'][m][0], gap, color=cc[r['label']], lw=1.4,
                label=f"{rung_label(r, two_line=False)}: "
                      f"median {np.median(gap):.0f} cm")
        r['design_gap'] = float(np.median(gap))
    lo.set_xlabel(f'seconds into each rung\'s own mid-flight {span:.0f} s window')
    lo.set_ylabel('distance from where\nthe law wanted it (cm)')
    lo.set_title('Design vs actual, the three windows laid over each other',
                 fontsize=10)
    lo.set_ylim(0, lo.get_ylim()[1] * 1.28)
    lo.legend(fontsize=8, ncol=len(runs), loc='upper left', framealpha=0.9)
    fig.suptitle(f'{spec["algo"]}: design vs actual — {span:.0f} s mid-flight',
                 y=0.97, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '2_expected_vs_actual.png'))
    plt.close(fig)


def fig_promise(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, axes = plt.subplots(1, len(runs), figsize=(3.9 * len(runs), 3.2),
                             squeeze=False, sharey=True)
    lab = {'H': 'coverage cost H', 'lattice_err': 'lattice error (m)'}[spec['promise']]
    for k, r in enumerate(runs):
        a = axes[0][k]
        if r.get('promise_logged') is not None:
            a.plot(r['t'], r['promise_logged'], color=cc[r['label']], lw=1.0,
                   alpha=0.35, ls=':',
                   label="as logged (recorder's clock)")
        if r['promise'] is not None:
            a.plot(r['t'], r['promise'], color=cc[r['label']], lw=1.6,
                   label='hardware' + (", algorithm's clock"
                                       if r.get('promise_logged') is not None else ''))
        if r['sim_promise'] is not None:
            a.plot(r['sim']['t'] - r['sim']['t'][0], r['sim_promise'], lw=1.6,
                   color=C.SIM, ls='--', label='simulation, same config')
        a.set_title(rung_label(r, two_line=False))
        a.set_xlabel('time since algorithm start (s)')
        if k == 0:
            a.set_ylabel(lab)
            a.legend(fontsize=7.5)
    fig.suptitle(f'{spec["algo"]}: the property the theorem promises',
                 y=1.03, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '3_promise.png'))
    plt.close(fig)


def fig_wobble_loops(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, axes = plt.subplots(1, len(runs), figsize=(3.3 * len(runs), 3.4),
                             squeeze=False)
    for k, r in enumerate(runs):
        a = axes[0][k]
        Z = r['P'][:, :, 0] + 1j * r['P'][:, :, 1]
        rip = C.bp(Z.real) + 1j * C.bp(Z.imag)
        for i, d in r['drones'].items():
            z = rip[:, d['j']] * 100
            a.plot(z.real, z.imag, lw=0.8 if d['real'] else 0.5,
                   color=cc[r['label']] if d['real'] else C.DESIGN,
                   alpha=0.9 if d['real'] else 0.55,
                   label=f"{i}{' (real)' if d['real'] else ''}")
        a.set_aspect('equal')
        a.set_title(f"{rung_label(r, two_line=False)}  —  ripple "
                    f"{max(d['wobble'] for d in r['drones'].values())*100:.1f} cm max",
                    fontsize=9.5)
        a.set_xlabel('x ripple (cm)')
        if k == 0:
            a.set_ylabel('y ripple (cm)')
        a.legend(fontsize=6.5, loc='upper right')
    fig.suptitle(f'{spec["algo"]}: the wobble itself (0.6–1.5 Hz band, '
                 f'slow motion removed)', y=1.03, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '4_wobble.png'))
    plt.close(fig)


def fig_delay(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    for r in runs:
        first_real = next((i for i, d in r['drones'].items() if d['real']), None)
        for i, d in r['drones'].items():
            ax[0].plot(d['lags'], d['xc'], lw=1.5 if d['real'] else 0.9,
                       color=cc[r['label']] if d['real'] else C.DESIGN,
                       alpha=1.0 if d['real'] else 0.5,
                       label=(r['label'] if d['real'] and i == first_real else None))
            if d['real']:
                ax[0].plot(d['lag'], d['xc_max'], C.MARKERS[d['j'] % 4], ms=7,
                           color=cc[r['label']])
    ax[0].axvline(0, color=C.GRID, lw=1)
    ax[0].set_xlabel('lag applied to the measured motion (s)')
    ax[0].set_ylabel('correlation with the commanded accel')
    ax[0].set_title('How late the drone executes the command')
    ax[0].legend(fontsize=7, ncol=1)

    for k, r in enumerate(runs):
        reals = [d for d in r['drones'].values() if d['real']]
        virts = [d for d in r['drones'].values() if not d['real']]
        if reals:
            ax[1].bar(k - 0.19, np.mean([d['lag'] for d in reals]), 0.36,
                      color=cc[r['label']], label='real' if k == 0 else None)
        if virts:
            ax[1].bar(k + 0.19, np.mean([d['lag'] for d in virts]), 0.36,
                      color=C.SIM, label='simulated (software path only)'
                      if k == 0 else None)
    ax[1].set_xticks(np.arange(len(runs)), [rung_label(r) for r in runs])
    ax[1].set_ylabel('measured loop delay τ (s)')
    ax[1].set_title('Delay does not change with the rung')
    ax[1].legend(fontsize=7.5, loc='lower right')
    fig.savefig(os.path.join(out, '5_delay.png'))
    plt.close(fig)


def fig_paths(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, axes = plt.subplots(1, len(runs), figsize=(3.3 * len(runs), 3.5),
                             squeeze=False)
    for k, r in enumerate(runs):
        a = axes[0][k]
        Ps = r['sim']['P']
        for j in range(Ps.shape[1]):
            a.plot(Ps[:, j, 0], Ps[:, j, 1], color=C.SIM, lw=1.0, ls='--',
                   alpha=0.8, label='simulation' if j == 0 else None)
        for i, d in r['drones'].items():
            a.plot(r['P'][:, d['j'], 0], r['P'][:, d['j'], 1],
                   color=cc[r['label']] if d['real'] else C.DESIGN,
                   lw=1.4 if d['real'] else 0.7, alpha=1.0 if d['real'] else 0.6,
                   label=f"{i}{' (real)' if d['real'] else ''}")
        a.set_aspect('equal')
        a.set_title(r['label'])
        a.set_xlabel('x (m)')
        if k == 0:
            a.set_ylabel('y (m)')
        a.legend(fontsize=6.5, loc='upper right')
    fig.suptitle(f'{spec["algo"]}: where they actually flew, against the same '
                 f'config in simulation', y=1.03, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '5_paths.png'))
    plt.close(fig)


def fig_command(runs, algo, spec, out):
    cc = _cols(runs, algo)
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.3))
    x = np.arange(len(runs))
    for k, r in enumerate(runs):
        for i, d in r['drones'].items():
            m = dict(ms=9 if d['real'] else 6,
                     mfc=cc[r['label']] if d['real'] else 'white',
                     mec=cc[r['label']], mew=1.6)
            ax[0].plot(k, d['clip'] * 100, C.MARKERS[d['j'] % 4], zorder=3, **m)
            ax[1].plot(k, d['tilt_med'], C.MARKERS[d['j'] % 4], zorder=3, **m)
            ax[2].plot(k, d['speed_med'], C.MARKERS[d['j'] % 4], zorder=3, **m)
    for a, lab, ttl in ((ax[0], 'ticks with the command clipped (%)', 'Saturation'),
                        (ax[1], 'median bank angle needed (°)', 'Tilt demanded'),
                        (ax[2], 'median speed (m/s)', 'How fast they moved')):
        a.set_xticks(x, [rung_label(r) for r in runs])
        a.set_ylabel(lab)
        a.set_title(ttl)
    ax[0].plot([], [], 'o', color=C.INK, label='real (filled)')
    ax[0].plot([], [], 'o', mfc='white', mec=C.INK, label='simulated')
    ax[0].legend(fontsize=7.5)
    fig.suptitle(f'{spec["algo"]}: what the law asked the hardware for',
                 y=1.04, fontsize=11, fontweight='bold')
    fig.savefig(os.path.join(out, '6_command.png'))
    plt.close(fig)


def table(runs, spec):
    print(f"\n{spec['algo']} ladder — {len(runs)} rung(s)\n")
    hdr = (f"{'rung':>5} {'drone':>7} {'':>5} {'k':>5} {'k·τ':>5} {'τ meas':>7} "
           f"{'ripple':>8} {'own':>7} {'period':>7} {'clip':>6} {'tilt':>6} {'v med':>6}")
    print(hdr); print('-' * len(hdr))
    for r in runs:
        for i, d in r['drones'].items():
            print(f"{r['label']:>5} {i:>7} {'REAL' if d['real'] else 'virt':>5} "
                  f"{d['k']:5.2f} {d['k_tau']:5.2f} {d['lag']:6.2f}s "
                  f"{d['wobble']*100:7.2f}cm {d['own']*100:6.2f}cm {d['period']:6.2f}s "
                  f"{d['clip']*100:5.0f}% {d['tilt_med']:5.1f}° {d['speed_med']:5.2f}"
                  f"{'   (τ weak, corr %.2f)' % d['xc_max'] if d['xc_max'] < 0.5 else ''}")
        if 'promise_settled' in r:
            print(f"{'':>5} {spec['promise']} settled {r['promise_settled']:.4f} "
                  f"vs sim {r['promise_sim_settled']:.4f} "
                  f"({r['promise_settled']/r['promise_sim_settled']-1:+.1%}), "
                  f"window {r['window']:.0f} s, shared-mode ripple "
                  f"{r['shared']*100:.2f} cm")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--algo', required=True, choices=sorted(LADDERS))
    ap.add_argument('--runs', help='override: "label=record:config,..."')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    spec = LADDERS[a.algo]
    runs_in = spec['runs']
    if a.runs:
        runs_in = []
        for chunk in a.runs.split(','):
            label, rest = chunk.split('=', 1)
            rec, cfg = rest.split(':', 1)
            runs_in.append((label, rec, cfg))

    out = a.out or os.path.join(C.ROOT, 'docs', 'figures', f'{a.algo}_ladder')
    os.makedirs(out, exist_ok=True)

    runs = [x for x in (analyse(l, r, c, spec) for l, r, c in runs_in) if x]
    # Order every figure the same way: closest to what the theory asks for
    # (lowest k·τ) on the left, furthest away on the right. The rung names run
    # in opposite directions for coverage (c1 slowest) and flocking (s1 is the
    # unrescaled law), so sorting on the names themselves would order the two
    # ladders inconsistently.
    runs.sort(key=lambda r: np.mean([d['k_tau'] for d in r['drones'].values()]))
    if not runs:
        print('No records to plot. Put the flight records in logs/hw/ first.')
        return
    table(runs, spec)

    fig_summary(runs, a.algo, spec, out)
    fig_expected_vs_actual(runs, a.algo, spec, out)
    fig_promise(runs, a.algo, spec, out)
    fig_wobble_loops(runs, a.algo, spec, out)
    fig_delay(runs, a.algo, spec, out)
    fig_command(runs, a.algo, spec, out)

    summary = {r['label']: {k: v for k, v in r.items()
                            if k in ('rec', 'cfg', 'window', 'shared',
                                     'design_gap',
                                     'promise_settled', 'promise_sim_settled',
                                     'cdist_max', 'd_min', 'vel_spread')}
               for r in runs}
    for r in runs:
        summary[r['label']]['drones'] = {
            i: {k: (None if isinstance(v, float) and np.isnan(v) else v)
                for k, v in d.items() if k not in ('lags', 'xc')}
            for i, d in r['drones'].items()}
    with open(os.path.join(out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'figures -> {out}')


if __name__ == '__main__':
    main()
