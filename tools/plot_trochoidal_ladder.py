#!/usr/bin/env python3
"""Plots for the trochoidal speed ladder.

The 15 Sep flights: rescale 6 -> 10 -> 14, all with max_accel 3.5 reaching the
drones (docs/PROJECT_AIM.md 11h).

    python3 tools/plot_trochoidal_ladder.py [--r6 <record>] [--out docs/figures/trochoidal_ladder]

Reads the records listed in RUNS from logs/hw/, prints a metrics table, and
writes the figures. Every number is computed from the raw rows over the clean
window (algorithm start + 2 s to the first drone at the 1.3 m edge, or to the
end of live data), as in analyse_trochoidal.py.
"""

import argparse
import copy
import math
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                 # noqa: E402
from scipy.signal import butter, sosfiltfilt, welch             # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyse_trochoidal as A                                  # noqa: E402
import sim_baseline as S                                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed', 'config')
LOGS = os.path.join(ROOT, 'logs', 'hw')

# 15 Sep 2026: the first ladder with max_accel 3.5 actually reaching the drones.
# (rung, record, config)
RUNS = [
    ('r2', 'trochoidalconsensus_20260916_145915.txt', 'testbed_fig4_r2.yaml'),
    ('r3', 'trochoidalconsensus_20260916_143323.txt', 'testbed_fig4_r3.yaml'),
    ('r6', 'trochoidalconsensus_20260915_151744.txt', 'testbed_fig4_r6.yaml'),
    ('r10', 'trochoidalconsensus_20260915_145550.txt', 'testbed_fig4_r10.yaml'),
    ('r14', 'trochoidalconsensus_20260915_143416.txt', 'testbed_fig4.yaml'),
]
CLAMP = 3.5

DT = 0.1          # the recorder's row interval
EDGE = 1.3        # m, end of the clean window
PATTERN_HZ = 0.4  # below this is the designed pattern (fast lap 8-19 s)
SHAKE_BAND = (0.6, 1.5)
G = 9.81
REAL = ('drone1', 'drone4')

INK = '#1f2328'
MUTED = '#6b7280'
GRID = '#e5e7eb'
DESIGN = '#9ca3af'
# slow to fast, light to dark
RUNG_COLOUR = {'r2': '#f0cf6b', 'r3': '#e0a93c', 'r4': '#d98c2b',
               'r6': '#cf6f1e', 'r10': '#b4441c', 'r14': '#8f2418'}
# evenly spaced slots, not the beta value: r2/r3/r4 would otherwise collide
RUNG_X = {'r2': 0, 'r3': 1, 'r4': 2, 'r6': 3, 'r10': 4, 'r14': 5}
RUNGS = []                 # the rungs actually plotted, filled in main()
DRONE_MARKER = {'drone1': 'o', 'drone4': 's'}
# rung -> the typical speed its designed pattern asks of the real drones (median,
# from simulation). Filled in main().
NAME = {}


def name(rung):
    return NAME.get(rung, rung)

plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
    'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': MUTED,
    'ytick.color': MUTED, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.6, 'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'savefig.dpi': 170, 'savefig.bbox': 'tight',
})


# crazyflie_node, as launched on 15 Sep (hardware_hybrid.launch.py passes geofence,
# max_lead and max_acceleration; max_velocity is left at the node's default)
CF_RATE = 25.0
CF_MAX_VEL = 0.7
CF_MAX_LEAD = 0.3
CF_GEOFENCE = 1.5


def reconstruct_setpoint(t, P, ids, i, law, clamp, t_eng, t_stop):
    """The position setpoint crazyflie_node streamed to real drone `i`, rebuilt
    from the log. Same steps as crazyflie_node._update_setpoint: integrate the
    clipped command at 25 Hz into a velocity and position, cap the speed, leash
    to the drone, clamp to the geofence. The command is the law evaluated on the
    logged states, with velocity as a backward difference (mocap_state_node's
    fit is ~45 ms late; this is ~50 ms). Returns times, positions, leashed share."""
    al, be, ka, Rm, adj = law
    j = ids.index(i)
    V = np.r_[[np.zeros(2)], np.diff(P[:, j], axis=0) / np.diff(t)[:, None]]
    rel = sum(w * (P[:, j] - P[:, ids.index(n)]) for n, w in adj[i].items())
    u = np.clip(-al * P[:, j] - be * V - ka * (rel @ Rm.T), -clamp, clamp)
    dt = 1.0 / CF_RATE
    taus = np.arange(t_eng, t_stop, dt)
    real = np.stack([np.interp(taus, t, P[:, j, ax]) for ax in (0, 1)], 1)
    k = np.searchsorted(t, taus, side='right') - 1
    pos, vel = real[0].copy(), np.zeros(2)
    out, leashed = np.empty((len(taus), 2)), 0
    for n in range(len(taus)):
        vel = vel + u[k[n]] * dt
        pos = pos + vel * dt
        sp = math.hypot(*vel)
        if sp > CF_MAX_VEL:
            vel *= CF_MAX_VEL / sp
        lead = pos - real[n]
        d = math.hypot(*lead)
        if d > CF_MAX_LEAD:
            pos = real[n] + lead * (CF_MAX_LEAD / d)
            vel *= 0.5
            leashed += 1
        for ax in (0, 1):
            if abs(pos[ax]) > CF_GEOFENCE:
                lim = math.copysign(CF_GEOFENCE, pos[ax])
                pos[ax] = lim
                if vel[ax] * lim > 0:
                    vel[ax] = 0.0
        out[n] = pos
    return taus, out, leashed / len(taus)


def lp(x, fc, axis=0):
    return sosfiltfilt(butter(4, fc, 'low', fs=1 / DT, output='sos'), x, axis=axis)


def bp(x, lo, hi, axis=0):
    return sosfiltfilt(butter(4, [lo, hi], 'band', fs=1 / DT, output='sos'), x, axis=axis)


def analyse(label, rec, cfgname, clamp=CLAMP):
    cfg = yaml.safe_load(open(os.path.join(CFG, cfgname)))
    ids = [d['id'] for d in cfg['drones']]
    al, be, ka, th, adj, _ = A.law(cfg)
    Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])

    cols, D = A.load_record(rec)
    t = D[:, 0]
    P = np.stack([np.stack([D[:, cols.index(f'x_{i}')], D[:, cols.index(f'y_{i}')]], 1)
                  for i in ids], 1)
    last = A.live_end(P)
    _, virt, _, _, t_start = A.classify(t, P, last, ids)
    # crazyflie_node engages on the first command: the row before any virtual
    # agent (an exact double integrator, frozen until then) first moves
    moved = np.nonzero(np.any(np.abs(P[:, virt] - P[0, virt]) > 0, axis=(1, 2)))[0]
    t_eng = t[max(moved[0] - 1, 0)] if len(moved) else t_start
    far = np.nonzero((np.abs(P[:last + 1]) > EDGE).any(axis=(1, 2))
                     & (t[:last + 1] > t_start + 1))[0]
    # A tilt over 90 deg means VICON lost the body, or the drone has landed. The
    # 16 Sep r2 record ends that way: flips at 201 s, a jump to y = -3.3 m (outside
    # the room), then frozen rows. Everything after the first flip is tracking
    # noise, not flight, so the window stops there.
    # Require half a second of it: a single row over 90 deg is a mocap glitch
    # (r6 has one at 64.8 s), while a real loss runs for hundreds of rows.
    flips = []
    for i in REAL:
        c = f'tilt_{i}'
        if c not in cols:
            continue
        bad = (D[:, cols.index(c)] > 90) & (t > t_start + 1)
        run = 0
        for k, v in enumerate(bad):
            run = run + 1 if v else 0
            if run >= 5:
                flips.append(t[k - 4])
                break
    t_stop = min([t[last]] + flips)
    hit = len(far) > 0 and t[far[0]] <= t_stop
    t_end = t[far[0]] if hit else t_stop
    t0 = t_start + 2.0

    tg = np.arange(t0, t_end, DT)
    Pg = np.stack([np.stack([np.interp(tg, t, P[:, j, ax]) for ax in (0, 1)], 1)
                   for j in range(len(ids))], 1)                  # n, drone, 2
    tilt = {i: np.interp(tg, t, D[:, cols.index(f'tilt_{i}')])
            for i in REAL if f'tilt_{i}' in cols}
    real = [ids.index(i) for i in REAL]

    # simulation of the same config, same clock (algorithm start = 0)
    _, ts, _, ph, _ = S.run(cfg, t_end - t_start + 0.5)
    Ps = np.array(ph)
    ks = (ts >= 2.0) & (ts <= t_end - t_start)
    sim_fast = A.fit_modes(ts[ks], Ps[ks, :, 0] + 1j * Ps[ks, :, 1])[1][0]

    # the design again, but started from where the fleet actually was when the
    # algorithm started (real drones where they hovered, virtual ones on their
    # marks), for the expected-vs-actual figure
    k_s = int(np.searchsorted(t, t_start - 0.5))
    cfg0 = copy.deepcopy(cfg)
    for n, d in enumerate(cfg0['drones']):
        d['initial_position'] = P[k_s, n].tolist()
        d['initial_velocity'] = [0.0, 0.0]
    _, ts0, _, ph0, _ = S.run(cfg0, t_end - t_start + 0.5)
    Ps0 = np.array(ph0)
    ks0 = (ts0 >= 2.0) & (ts0 <= t_end - t_start)

    # pattern size: fleet RMS distance from the pattern centre, shake removed
    size = np.sqrt(np.mean(np.sum(lp(Pg, PATTERN_HZ) ** 2, axis=2), axis=1))
    sim_size = np.sqrt(np.mean(np.sum(Ps[ks] ** 2, axis=2), axis=1))
    # designed size, speed and tilt all come from the design as started from
    # the real start: on 15 Sep the two real drones sat on each other's marks
    ref = np.sqrt(np.mean(np.sum(Ps0[ks0] ** 2, axis=2), axis=1)).mean()
    tau = tg - t_start
    rate = np.polyfit(tau, np.log(size), 1)[0]
    hw_modes = A.fit_modes(tg, Pg[..., 0] + 1j * Pg[..., 1])

    # acceleration the drones achieved, and the command they were sent
    V = np.gradient(Pg, DT, axis=0)
    Acc = np.gradient(V, DT, axis=0)
    out = dict(cfg=cfg, label=label, clamp=clamp, beta=be, hit=hit, window=t_end - t0,
               to_edge=(t_end - t_start) if hit else None, live=t_end - t_start,
               fast_T=sim_fast, rate=rate, per_lap=math.exp(rate * sim_fast),
               mode_growth=hw_modes[1][1], mode_r2=hw_modes[2],
               tau=tau, size=size / ref, sim_tau=ts[ks], sim_size=sim_size / ref, sim_P=Ps[ks],
               start_tau=ts0, start_P=Ps0,
               tg=tg, Pg=Pg, real=real, drones={})
    for i, j in zip(REAL, real):
        rel = sum(w * (Pg[:, j] - Pg[:, ids.index(n)]) for n, w in adj[i].items())
        u = -al * Pg[:, j] - be * V[:, j] - ka * (rel @ Rm.T)
        uc = np.clip(u, -clamp, clamp)
        lags = np.arange(-0.4, 0.81, 0.01)
        xc = []
        for lag in lags:
            ai = np.stack([np.interp(tg + lag, tg, Acc[:, j, ax]) for ax in (0, 1)], 1)
            m = (tg + lag >= tg[0]) & (tg + lag <= tg[-1])
            xc.append(np.sum(uc[m] * ai[m]) / np.sqrt(np.sum(uc[m] ** 2) * np.sum(ai[m] ** 2)))
        xc = np.array(xc)
        z = Pg[:, j, 0] + 1j * Pg[:, j, 1]
        zs = bp(z.real, *SHAKE_BAND) + 1j * bp(z.imag, *SHAKE_BAND)
        f, psd = welch(z - z.mean(), fs=1 / DT, nperseg=min(256, len(z)),
                       return_onesided=False, detrend='linear')
        band = (np.abs(f) >= SHAKE_BAND[0]) & (np.abs(f) <= SHAKE_BAND[1])
        pk = f[band][np.argmax(psd[band])]
        ccw = psd[band & (f > 0)].sum() / psd[band].sum()
        ulp, uclp = lp(u, PATTERN_HZ), lp(uc, PATTERN_HZ)
        sim_acc = np.gradient(np.gradient(Ps0[ks0, j], DT, axis=0), DT, axis=0)
        out['drones'][i] = dict(
            sat=np.mean(np.any(np.abs(u) > clamp, axis=1)),
            u_med=np.median(np.hypot(*u.T)),
            a_med=np.median(np.hypot(*Acc[:, j].T)),
            lags=lags, xc=xc, lag=lags[np.argmax(xc)], xc_max=xc.max(),
            shake_T=1 / abs(pk), shake_rms=np.sqrt(np.mean(np.abs(zs) ** 2)),
            circ=max(ccw, 1 - ccw), sense='CCW' if ccw > 0.5 else 'CW',
            f=np.fft.fftshift(f), psd=np.fft.fftshift(psd),
            tilt_flown=np.degrees(np.arctan(np.hypot(*Acc[:, j].T) / G)),
            tilt_meas=tilt.get(i),
            tilt_design=np.percentile(np.degrees(np.arctan(np.hypot(*sim_acc.T) / G)), 95),
            passed=np.sum(uclp * ulp) / np.sum(ulp * ulp),
            speed=np.median(np.hypot(*V[:, j].T)),
            speed_design=np.median(np.hypot(*np.gradient(Ps0[ks0, j], DT, axis=0).T)),
        )
        tsp, sp, leash = reconstruct_setpoint(t, P, ids, i, (al, be, ka, Rm, adj), clamp,
                                              t_eng, t[last])
        sp_g = np.stack([np.interp(tg, tsp, sp[:, ax]) for ax in (0, 1)], 1)
        gaps = []
        for L in np.arange(0, 0.61, 0.02):
            ok = tg + L <= tg[-1]
            ah = np.stack([np.interp(tg[ok] + L, tg, Pg[:, j, ax]) for ax in (0, 1)], 1)
            gaps.append((np.sqrt(np.mean(np.sum((ah - sp_g[ok]) ** 2, axis=1))), L))
        out['drones'][i].update(sp=sp_g, sp_leash=leash,
                                sp_gap=np.sqrt(np.mean(np.sum((sp_g - Pg[:, j]) ** 2, axis=1))),
                                sp_lag=min(gaps)[1], sp_gap_lagged=min(gaps)[0])
    return out


def table(runs):
    print('\nrung  clean   edge   size x/lap | drone   clipped  delay  shake period  shake size  '
          'tilt meas/path/design   speed real/design')
    for r in runs:
        print(f"{r['label']:4s} {r['window']:5.0f}s  {'hit' if r['hit'] else 'never':5s}  x{r['per_lap']:5.2f}")
        for i, v in r['drones'].items():
            meas = f"{np.nanmedian(v['tilt_meas']):4.1f}" if v['tilt_meas'] is not None else '  --'
            print(f"{'':30s}{i}  {v['sat'] * 100:4.0f}%   {v['lag']:.2f}s     {v['shake_T']:.2f}s"
                  f"      {v['shake_rms'] * 100:5.1f} cm    {meas} / {np.median(v['tilt_flown']):4.1f} / "
                  f"{v['tilt_design']:.1f}       {v['speed']:.2f} / {v['speed_design']:.2f} m/s")

    print('\ncommanded position (setpoint rebuilt from the log) vs where the drone was')
    for r in runs:
        for i, v in r['drones'].items():
            print(f"  {r['label']:4s} {i}: RMS gap {v['sp_gap'] * 100:4.1f} cm; allowing for the "
                  f"drone's lag ({v['sp_lag']:.2f} s) {v['sp_gap_lagged'] * 100:4.1f} cm; "
                  f"leash engaged {v['sp_leash'] * 100:3.0f}% of ticks")


# -- figures ------------------------------------------------------------------

def _rung_axis(ax):
    xs = [RUNG_X[k] for k in RUNGS]
    ax.set_xticks(xs)
    ax.set_xticklabels(RUNGS)
    ax.set_xlim(min(xs) - 0.7, max(xs) + 0.7)


def _dots(ax, runs, key, scale=1.0):
    for r in runs:
        for n, (i, v) in enumerate(r['drones'].items()):
            ax.plot(RUNG_X[r['label']] + (n - 0.5) * 0.17, v[key] * scale,
                    marker=DRONE_MARKER[i], color=RUNG_COLOUR[r['label']], ms=7, ls='none')


def _design_bars(ax, runs, key, scale=1.0):
    for r in runs:
        val = max(v[key] for v in r['drones'].values()) * scale
        x = RUNG_X[r['label']]
        ax.plot([x - 0.42, x + 0.42], [val, val], color=DESIGN, lw=3, solid_capstyle='butt')


def fig_summary(runs, out):
    fig, axs = plt.subplots(2, 3, figsize=(11, 6.8))

    ax = axs[0, 0]
    for r in runs:
        ax.bar(RUNG_X[r['label']], r['per_lap'], width=0.62, color=RUNG_COLOUR[r['label']])
        ax.text(RUNG_X[r['label']], r['per_lap'] + 0.04, f"×{r['per_lap']:.2f}",
                ha='center', va='bottom', fontsize=8.5, color=INK)
    ax.axhline(1, color=INK, lw=0.8, ls='--')
    ax.axhline(2, color=MUTED, lw=0.6, ls=':')
    ax.text(0.98, 0.84, 'would double every lap', transform=ax.transAxes, ha='right',
            va='bottom', color=MUTED, fontsize=7.5)
    ax.set_ylim(0, 2.4)
    ax.set_title('Pattern size change per lap\n(1 = holds its size)')
    ax.set_ylabel('× per lap')

    ax = axs[0, 1]
    _design_bars(ax, runs, 'speed_design')
    _dots(ax, runs, 'speed')
    ax.set_title('How fast the real drones moved\n(grey = what the pattern needs)')
    ax.set_ylabel('median speed, m/s')
    ax.set_ylim(0, 1.3)

    ax = axs[0, 2]
    _design_bars(ax, runs, 'tilt_design')
    for r in runs:
        for n, (i, v) in enumerate(r['drones'].items()):
            if v['tilt_meas'] is not None:
                ax.plot(RUNG_X[r['label']] + (n - 0.5) * 0.17, np.nanmedian(v['tilt_meas']),
                        marker=DRONE_MARKER[i], color=RUNG_COLOUR[r['label']], ms=7, ls='none')
    ax.set_title('How far the real drones tilted\n(grey = what the pattern needs)')
    ax.set_ylabel('median tilt, degrees (VICON)')
    ax.set_ylim(0, 36)

    ax = axs[1, 0]
    _dots(ax, runs, 'shake_rms', 100)
    ax.set_title('Size of the fast shake')
    ax.set_ylabel('cm (RMS)')
    ax.set_ylim(0, 22)

    ax = axs[1, 1]
    taus = [v['lag'] for r in runs for v in r['drones'].values()]
    lo, hi = 4 * min(taus), 4 * max(taus)
    ax.axhspan(lo, hi, color=DESIGN, alpha=0.35, lw=0)
    ax.text(0.02, 0.55, f'grey band: 4 × the drones\' delay ({min(taus):.2f}–{max(taus):.2f} s)',
            transform=ax.transAxes, color=MUTED, fontsize=7.5, va='top')
    _dots(ax, runs, 'shake_T')
    ax.set_title('Time for one shake\n(same at every speed)')
    ax.set_ylabel('s')
    ax.set_ylim(0, 1.8)

    ax = axs[1, 2]
    _dots(ax, runs, 'sat', 100)
    ax.set_title(f'Command maxed out\n(asked for more than {CLAMP} m/s²)')
    ax.set_ylabel('% of the time')
    ax.set_ylim(0, 105)

    for ax in axs.flat:
        _rung_axis(ax)
    from matplotlib.lines import Line2D
    h = [Line2D([], [], marker='o', color=MUTED, ls='none', ms=7),
         Line2D([], [], marker='s', color=MUTED, ls='none', ms=7),
         Line2D([], [], color=DESIGN, lw=3)]
    fig.legend(h, ['drone1', 'drone4', 'what the designed pattern needs (simulation)'],
               loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.01))
    fig.text(0.5, -0.048, 'designed pattern speed:   '
             + '    '.join(f'{k} = {NAME[k]}' for k in RUNGS),
             ha='center', color=MUTED, fontsize=8.5)
    fig.suptitle(f'Trochoidal speed ladder, 15–16 Sep: max_accel {CLAMP} throughout',
                 x=0.01, ha='left', fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    fig.savefig(os.path.join(out, '1_summary.png'))
    plt.close(fig)


def fig_size(runs, out):
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for r in runs:
        ax.plot(r['tau'] / r['fast_T'], r['size'], color=RUNG_COLOUR[r['label']], lw=1.8,
                label=name(r['label']))
        if r['hit']:
            ax.plot(r['tau'][-1] / r['fast_T'], r['size'][-1], 'x', color=RUNG_COLOUR[r['label']],
                    ms=9, mew=2.2, label=f"{name(r['label'])}: a drone passed 1.3 m here")
    ax.axhline(1, color=DESIGN, lw=3, label='designed size')
    ax.axhline(2, color=MUTED, lw=0.6, ls=':')
    ax.set_yscale('log', base=2)
    ax.set_yticks([0.5, 1, 2, 4])
    ax.set_yticklabels(['half', 'designed', 'double', '4×'])
    ax.set_ylim(0.5, 5)
    ax.set_xlabel('laps since the algorithm started')
    ax.set_ylabel('pattern size')
    ax.legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8.5,
              title='designed pattern speed', title_fontsize=8.5)
    fig.suptitle('Pattern size over time (shake filtered out)', x=0.01, ha='left',
                 fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(out, '2_pattern_size.png'))
    plt.close(fig)


def fig_shake(runs, out):
    fig = plt.figure(figsize=(11, 6.6))
    gs = fig.add_gridspec(2, len(runs), height_ratios=[1, 0.9])
    for k, r in enumerate(runs):
        ax = fig.add_subplot(gs[0, k])
        j = r['real'][0]
        mid = len(r['tg']) // 2
        s = slice(mid - 40, mid + 40)                             # 8 s
        p = r['Pg'][s, j]
        pat = lp(r['Pg'][:, j], PATTERN_HZ)[s]
        c = p.mean(0)
        ax.plot(pat[:, 0] - c[0], pat[:, 1] - c[1], color=DESIGN, lw=3)
        ax.plot(p[:, 0] - c[0], p[:, 1] - c[1], color=RUNG_COLOUR[r['label']], lw=1.1)
        ax.set_aspect('equal')
        ax.set_xlim(-0.45, 0.45)
        ax.set_ylim(-0.45, 0.45)
        ax.set_title(name(r['label']), fontsize=10)
        ax.set_xlabel('x, m')
        if k == 0:
            ax.set_ylabel('y, m')
    fig.text(0.01, 0.955, 'drone1, 8 s mid-flight. Colour: the path it actually flew. '
             'Grey: the same path with the shake filtered out.', color=MUTED, fontsize=8.5)
    ax = fig.add_subplot(gs[1, :])
    for r in runs:
        psd = np.mean([v['psd'] for v in r['drones'].values()], axis=0)
        f = next(iter(r['drones'].values()))['f']
        pos = f > 0
        fold = np.array([psd[f == fp].sum() + psd[np.isclose(f, -fp)].sum() for fp in f[pos]])
        ax.plot(f[pos], fold * 1e4, color=RUNG_COLOUR[r['label']], lw=1.4, label=name(r['label']))
    ax.set_yscale('log')
    ax.set_xlim(0, 2.5)
    ax.set_ylim(1e-3, 1e5)
    taus = [v['lag'] for r in runs for v in r['drones'].values()]
    f0 = 1 / (4 * np.mean(taus))
    ax.axvline(f0, color=INK, lw=0.8, ls=':')
    ax.text(f0 + 0.02, 3e4, f'the shake: once every {1 / f0:.2f} s', fontsize=8, color=INK)
    ax.text(0.03, 3e4, 'the pattern', fontsize=8, color=INK)
    ax.set_xlabel('frequency, Hz (how many times per second)')
    ax.set_ylabel('amount of motion')
    ax.set_title('Where the motion is: the slow pattern (left) and one sharp shake peak',
                 loc='left')
    ax.legend(loc='upper right')
    fig.suptitle('The fast shake', x=0.01, y=1.0, ha='left', fontsize=12,
                 fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(out, '3_shake.png'))
    plt.close(fig)


def fig_delay(runs, out):
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0), gridspec_kw=dict(width_ratios=[1.5, 1]))
    ax = axs[0]
    for r in runs:
        for i, v in r['drones'].items():
            ax.plot(v['lags'], v['xc'], color=RUNG_COLOUR[r['label']], lw=1.2,
                    ls='-' if i == 'drone1' else '--', label=f"{name(r['label'])} {i}")
    ax.axvline(0, color=INK, lw=0.6)
    ax.set_xlabel('delay, s')
    ax.set_ylabel('match between command and motion')
    ax.set_title('The drones do what they are told, about 0.27 s late', loc='left')
    ax.legend(fontsize=7.5, ncol=2, loc='upper left')
    ax = axs[1]
    for r in runs:
        for i, v in r['drones'].items():
            ax.plot(4 * v['lag'], v['shake_T'], marker=DRONE_MARKER[i],
                    color=RUNG_COLOUR[r['label']], ms=7, ls='none')
    ax.plot([0.8, 1.5], [0.8, 1.5], color=MUTED, lw=0.8, ls='--')
    ax.text(1.45, 1.39, 'shake time = 4 × delay', ha='right', va='top', color=MUTED, fontsize=8,
            rotation=45, rotation_mode='anchor')
    ax.set_xlim(0.8, 1.5)
    ax.set_ylim(0.8, 1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('4 × measured delay, s')
    ax.set_ylabel('measured time for one shake, s')
    ax.set_title('That delay predicts the shake', loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(out, '4_delay.png'))
    plt.close(fig)


def fig_tilt(runs, out):
    fig, axs = plt.subplots(len(runs), 1, figsize=(11, 2.3 * len(runs)), sharex=True)
    axs = np.atleast_1d(axs)
    for ax, r in zip(axs, runs):
        v = r['drones']['drone1']
        m = (r['tau'] >= 40) & (r['tau'] <= 60)
        ax.plot(r['tau'][m], v['tilt_meas'][m], color=INK, lw=1.2, label='measured by VICON')
        ax.plot(r['tau'][m], v['tilt_flown'][m], color=RUNG_COLOUR[r['label']], lw=1.2,
                label='calculated from the path it flew')
        ax.axhline(v['tilt_design'], color=DESIGN, lw=3, label='what the designed pattern needs')
        ax.set_ylabel('tilt, °')
        ax.set_title(f"{name(r['label'])}, drone1", loc='left', fontsize=9)
        ax.set_ylim(0, 60)
    axs[0].legend(ncol=3, fontsize=8, loc='upper left')
    axs[-1].set_xlabel('s since the algorithm started')
    fig.suptitle('Tilt: measured matches what the shaky path needs',
                 x=0.01, ha='left', fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(out, '5_tilt.png'))
    plt.close(fig)


def _restarted_design(r, t0, secs):
    """The design, restarted from where the fleet actually was at t0 (positions
    and velocities with the fast loops filtered out). Times on the flight clock."""
    k0 = int(np.searchsorted(r['tau'], t0))
    P = lp(r['Pg'], PATTERN_HZ)
    V = np.gradient(P, DT, axis=0)
    cfg = copy.deepcopy(r['cfg'])
    for n, d in enumerate(cfg['drones']):
        d['initial_position'] = P[k0, n].tolist()
        d['initial_velocity'] = V[k0, n].tolist()
    _, ts, _, ph, _ = S.run(cfg, secs + DT)
    return r['tau'][k0] + ts, np.array(ph)


def fig_expected(runs, out, where='start', ref='setpoint', secs=20.0):
    """drone1: a reference path against where it actually was.
    ref 'setpoint': the position crazyflie_node commanded, rebuilt from the log.
    ref 'design': the designed pattern -- from the real start ('start'), or
    restarted from where the fleet actually was ('middle')."""
    fig, axs = plt.subplots(len(runs), 2, figsize=(11, 3.3 * len(runs)),
                            gridspec_kw=dict(width_ratios=[1, 2.4]))
    axs = np.atleast_2d(axs)
    if ref == 'setpoint':
        lab, ekw = 'commanded position (setpoint)', dict(color=INK, lw=1.0)
    else:
        lab, ekw = 'expected (design)', dict(color=DESIGN, lw=3.5)
    for row, r in zip(axs, runs):
        j = r['real'][0]
        c = RUNG_COLOUR[r['label']]
        if where == 'start':
            a, b = 2.0, 2.0 + secs
        else:
            mid = 0.5 * (r['tau'][0] + r['tau'][-1])
            a, b = mid - secs / 2, mid + secs / 2
        if ref == 'setpoint':
            et, ev = r['tau'], r['drones']['drone1']['sp']
        elif where == 'start':
            et, ev = r['start_tau'], r['start_P'][:, j]
        else:
            et, eP = _restarted_design(r, a, secs)
            ev = eP[:, j]
        m = (r['tau'] >= a) & (r['tau'] <= b)
        ms = (et >= a) & (et <= b)
        act, exp_ = r['Pg'][m, j], ev[ms]

        ax = row[0]
        ax.plot(exp_[:, 0], exp_[:, 1], label=lab, **ekw)
        ax.plot(act[:, 0], act[:, 1], color=c, lw=1.1, alpha=0.85, label='actual (VICON)')
        ax.set_aspect('equal')
        ax.set_xlim(-1.1, 1.1)
        ax.set_ylim(-1.1, 1.1)
        ax.set_xlabel('x, m')
        ax.set_ylabel('y, m')
        ax.set_title(f"{name(r['label'])}: path from above", loc='left', fontsize=9)

        ax = row[1]
        ax.plot(et[ms], exp_[:, 0], label=lab, **ekw)
        ax.plot(r['tau'][m], act[:, 0], color=c, lw=1.3, alpha=0.85, label='actual (VICON)')
        ax.set_xlim(a, b)
        ax.set_ylim(-1.1, 1.1)
        ax.set_ylabel('x position, m')
        ax.set_title(f"{name(r['label'])}: x position over time "
                     f"(designed loop takes {r['fast_T']:.0f} s)", loc='left', fontsize=9)
        ax.legend(loc='upper right', fontsize=8, ncol=2)
    axs[-1, 1].set_xlabel('s since the algorithm started')
    span = f'first {secs:.0f} s of each flight' if where == 'start' else \
        f'{secs:.0f} s from the middle of each flight'
    if ref == 'setpoint':
        title = (f'drone1: commanded vs actual position, {span}\n'
                 'commanded = the setpoint crazyflie_node streamed, rebuilt from the log '
                 '(the setpoint itself is not recorded)')
        fname = '6_commanded_vs_actual.png' if where == 'start' else '7_commanded_vs_actual_middle.png'
    elif where == 'start':
        title = (f'drone1: design vs actual position, {span}\n'
                 'design started from where the drones actually were when the algorithm started')
        fname = '8_design_vs_actual.png'
    else:
        title = (f'drone1: design vs actual position, {span}\n'
                 'design restarted from where the drones actually were at the start of this '
                 'window (fast loops filtered out)')
        fname = '9_design_vs_actual_middle.png'
    fig.suptitle(title, x=0.01, ha='left', fontsize=11, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(out, fname))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--r6', help='the 15 Sep rescale-6 record, until it is added to RUNS')
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'figures', 'trochoidal_ladder'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = []
    for label, rec, cfgname in RUNS:
        rec = a.r6 if label == 'r6' and a.r6 else rec
        path = rec if rec and os.path.isabs(rec) else os.path.join(LOGS, rec or '')
        if not rec or not os.path.isfile(path):
            print(f'  {label}: no record -- left out' + (' (pass --r6 <file>)' if label == 'r6' else ''))
            continue
        runs.append(analyse(label, path, cfgname))
    RUNGS.extend(r['label'] for r in runs)
    for r in runs:
        NAME[r['label']] = f"{np.median([v['speed_design'] for v in r['drones'].values()]):.2f} m/s"
    table(runs)
    fig_summary(runs, a.out)
    fig_size(runs, a.out)
    fig_shake(runs, a.out)
    fig_delay(runs, a.out)
    fig_tilt(runs, a.out)
    for where in ('start', 'middle'):
        for ref in ('setpoint', 'design'):
            fig_expected(runs, a.out, where=where, ref=ref)
    print(f'\nfigures in {a.out}')


if __name__ == '__main__':
    main()
