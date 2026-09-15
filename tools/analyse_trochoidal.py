#!/usr/bin/env python3
"""Trochoidal flight metrics: laps before the pattern doubles, and tilt.

    python3 tools/analyse_trochoidal.py --config <yaml the flight used> <record.txt>

The recorder's own trochoidal summary fits across the whole record, geofence
tracing and frozen tail included, so its numbers cannot be quoted
(docs/PROJECT_AIM.md section 11c). This scores only the clean window: from the
moment the algorithm starts (the virtual drones begin to move) to the moment
any drone reaches the geofence edge.

It reports
  - the two modes (period and growth rate), fitted jointly over the fleet, next
    to simulation of the same config over the same window length;
  - how long the pattern takes to double in size, in seconds and in fast laps;
  - time from algorithm start to the geofence edge;
  - each real drone's tilt (records with tilt_ columns, from 2026-09-15 on),
    against the point-mass prediction atan(|a|/g) from simulation;
  - which gains actually flew, by replaying the virtual drones through the
    control law driven by the logged real ones (section 11b).

numpy is enough; scipy, if present, sharpens the mode fit.
"""

import argparse
import math
import os
import re
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_baseline as S                                        # noqa: E402
from drone_testbed.dynamics.double_integrator import step       # noqa: E402

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

G = 9.81


def load_record(path):
    cols, rows = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith('#'):
                m = re.match(r'#\s+(\d+)\s+(\S+)\s*$', line)
                if m:
                    cols.append(m.group(2))
                continue
            parts = line.split()
            if parts:
                try:
                    rows.append([float(p) for p in parts])
                except ValueError:
                    pass
    return cols, np.array(rows)


def law(cfg):
    p = cfg['algorithm']['params']
    adj = {}
    for i, nbrs in (p.get('adjacency') or {}).items():
        adj[i] = dict(nbrs) if isinstance(nbrs, dict) else {n: 1.0 for n in nbrs}
    return (float(p['alpha']), float(p['beta']), float(p['kappa']),
            float(p['theta']), adj, float(p.get('max_accel', 0.5)))


# -- windows -----------------------------------------------------------------

def live_end(P):
    """Last row that differs from its predecessor. The recorder keeps writing
    the last state after the feed dies; a real drone never repeats a row."""
    flat = P.reshape(len(P), -1)
    changed = np.r_[True, np.any(np.abs(np.diff(flat, axis=0)) > 0, axis=1)]
    return int(np.nonzero(changed)[0].max())


def classify(t, P, last, ids):
    """Real drones move from takeoff (1-3 s in); virtual ones hold until the
    algorithm starts (~12 s). A virtual agent that never started is exactly
    constant; a real one that never took off still shows VICON noise."""
    first = []
    for j in range(len(ids)):
        d = np.hypot(*(P[:last + 1, j, :] - P[0, j, :]).T)
        k = np.nonzero(d > 0.02)[0]
        first.append(t[k[0]] if len(k) else np.nan)
    earliest = np.nanmin(first)
    real, virt, grounded = [], [], []
    for j, f in enumerate(first):
        if np.isnan(f):
            (grounded if np.std(P[:last + 1, j, :]) > 0 else virt).append(j)
        elif f > earliest + 5.0:
            virt.append(j)
        else:
            real.append(j)
    started = [first[j] for j in virt if not np.isnan(first[j])]
    t_start = float(np.median(started)) if started else None
    return real, virt, grounded, first, t_start


# -- the two modes ---------------------------------------------------------------

def _resid(tau, Z, s):
    B = np.stack([np.ones_like(tau, dtype=complex),
                  np.exp(s[0] * tau), np.exp(s[1] * tau)], 1)
    C, *_ = np.linalg.lstsq(B, Z, rcond=None)
    return Z - B @ C


def fit_modes(t, Z):
    """Two shared complex exponentials plus a centre, over every drone at once:
    the fleet's two closed-loop eigenvalue pairs. Returns (slow, fast), each
    (period s, growth rate 1/s, 'CW'|'CCW'), and R^2."""
    tau = t - t[0]
    best = (np.inf, None)
    for w1 in np.arange(-0.8, 0.801, 0.01):
        for w2 in np.arange(-2.0, 2.001, 0.02):
            if abs(w2) <= abs(w1) + 0.02:
                continue
            r = np.sum(np.abs(_resid(tau, Z, (1j * w1, 1j * w2))) ** 2)
            if r < best[0]:
                best = (r, (w1, w2))
    x = np.array([0.0, best[1][0], 0.0, best[1][1]])
    if least_squares is not None:
        def f(p):
            R = _resid(tau, Z, (p[0] + 1j * p[1], p[2] + 1j * p[3]))
            return np.r_[R.real.ravel(), R.imag.ravel()]
        x = least_squares(f, x).x
    R = _resid(tau, Z, (x[0] + 1j * x[1], x[2] + 1j * x[3]))
    r2 = 1 - np.sum(np.abs(R) ** 2) / np.sum(np.abs(Z - Z.mean(0)) ** 2)
    modes = sorted([(x[0], x[1]), (x[2], x[3])], key=lambda m: abs(m[1]))
    out = [(2 * np.pi / abs(w), lam, 'CCW' if w > 0 else 'CW') for lam, w in modes]
    return out[0], out[1], r2


# -- which gains flew ----------------------------------------------------------

def replay(t, P, ids, virt, cfg, scale, ts, te, clamp):
    """Fly the virtual agents again through the law at gains scaled by
    `scale` (time rescale: alpha s^2, beta s, kappa s^2), with the real
    agents as logged. RMS miss against the logged virtual paths, metres."""
    al, be, ka, th, adj, _ = law(cfg)
    al, be, ka = al * scale ** 2, be * scale, ka * scale ** 2
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    dt = 1.0 / float(cfg['simulation'].get('control_rate', 10.0))
    vid = [ids[j] for j in virt]
    at = lambda j, tq: np.array([np.interp(tq, t, P[:, j, 0]), np.interp(tq, t, P[:, j, 1])])
    st = {i: np.r_[at(ids.index(i), ts), 0.0, 0.0] for i in vid}
    tt, err = ts, []
    while tt < te:
        pos = {i: (st[i][:2] if i in st else at(ids.index(i), tt)) for i in ids}
        for i in vid:
            rel = sum((w * (pos[i] - pos[n]) for n, w in adj.get(i, {}).items()),
                      np.zeros(2))
            a = np.clip(-al * st[i][:2] - be * st[i][2:] - ka * (R @ rel), -clamp, clamp)
            s = step(st[i], a, dt)
            sp = math.hypot(*s[2:])
            if sp > 0.7:                      # drone_node's velocity clamp
                s[2:] *= 0.7 / sp
            st[i] = s
        tt += dt
        err += [np.hypot(*(st[i][:2] - at(ids.index(i), tt))) for i in vid]
    return float(np.sqrt(np.mean(np.square(err))))


def which_gains(t, P, ids, virt, cfg, t_start, t_end, clamp):
    te = min(t_end, t_start + 60.0)
    grid = lambda scales, starts: min(
        (replay(t, P, ids, virt, cfg, sc, ts, te, clamp), sc, ts)
        for sc in scales for ts in starts)
    e1 = grid([1.0], np.arange(t_start - 2.0, t_start + 0.01, 0.2))
    coarse = grid(np.arange(0.4, 1.61, 0.1), np.arange(t_start - 2.0, t_start + 0.01, 0.4))
    fine = grid(np.arange(coarse[1] - 0.1, coarse[1] + 0.101, 0.01),
                np.arange(coarse[2] - 0.4, coarse[2] + 0.41, 0.1))
    return e1[0], fine[1], fine[0]


# -- report --------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('record')
    ap.add_argument('--config', required=True, help='the yaml the flight used')
    ap.add_argument('--edge', type=float, default=1.3,
                    help='clean window ends when any drone passes this |x| or |y| (m)')
    ap.add_argument('--virtual-clamp', type=float, default=None,
                    help="drone_node's acceleration clamp for the replay; default "
                         "the config's max_accel (use 0.5 for flights before 2026-09-15)")
    a = ap.parse_args()

    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    ids = [d['id'] for d in cfg['drones']]
    cols, D = load_record(a.record)
    t = D[:, 0]
    P = np.stack([np.stack([D[:, cols.index(f'x_{i}')], D[:, cols.index(f'y_{i}')]], 1)
                  for i in ids], 1)
    last = live_end(P)
    real, virt, grounded, first, t_start = classify(t, P, last, ids)
    name = lambda js: ', '.join(ids[j] for j in js) or '-'

    print(f'\n{os.path.basename(a.record)}   (config {os.path.basename(a.config)})')
    print(f'  live data to {t[last]:.1f} s of {t[-1]:.1f} s '
          f'(frozen tail {100 * (len(t) - 1 - last) / len(t):.0f}%)')
    print(f'  real: {name(real)}   virtual: {name(virt)}'
          + (f'   real but never moved: {name(grounded)}' if grounded else ''))
    if t_start is None or not real:
        print('  cannot find the algorithm start (no virtual drone moved) -- nothing to score')
        return
    far = np.nonzero((np.abs(P[:last + 1]) > a.edge).any(axis=(1, 2))
                     & (t[:last + 1] > t_start + 1))[0]
    t_end = t[far[0]] if len(far) else t[last]
    t0 = t_start + 2.0
    L = t_end - t0
    print(f'  algorithm started ~{t_start:.1f} s; clean window {t0:.1f}-{t_end:.1f} s ({L:.0f} s)')
    if len(far):
        print(f'  start -> first drone at the {a.edge} m edge: {t_end - t_start:.1f} s')
    else:
        print(f'  never reached the {a.edge} m edge while live')

    al, be, ka, th, adj, max_acc = law(cfg)
    _, ts, _, ph, _ = S.run(cfg, 2.0 + L + 0.2)
    Ps = np.array(ph)
    ks = (ts >= 2.0) & (ts <= 2.0 + L)

    if L < 20:
        print('\n  clean window under 20 s -- too short for the mode fit')
    else:
        k = (t >= t0) & (t <= t_end)
        hw = fit_modes(t[k], P[k, :, 0] + 1j * P[k, :, 1])
        sim = fit_modes(ts[ks], Ps[ks, :, 0] + 1j * Ps[ks, :, 1])
        print('\n  MODES (joint fit over the fleet)')
        print('    mode   period hw   period sim   diff   growth hw     size doubles every')
        for lab, h, s in (('slow', hw[0], sim[0]), ('fast', hw[1], sim[1])):
            dbl = math.log(2) / h[1] if h[1] > 0 else float('inf')
            laps = dbl / hw[1][0]
            print(f'    {lab:5s} {h[0]:8.2f} s {s[0]:10.2f} s {100 * (h[0] / s[0] - 1):+6.0f}%'
                  f'   {h[1]:+.4f} /s   ' + (f'{dbl:6.1f} s = {laps:4.1f} fast laps'
                                             if h[1] > 0 else 'not growing'))
        print(f'    period ratio {hw[0][0] / hw[1][0]:.2f} (sim {sim[0][0] / sim[1][0]:.2f})'
              f'   fit R^2 {hw[2]:.3f} (sim {sim[2]:.3f})')
        if L < hw[0][0]:
            print(f'    (window shorter than one slow period -- treat the slow mode as rough)')

    # tilt
    k = (t >= t0) & (t <= t_end)
    acc = np.gradient(np.gradient(Ps[ks], ts[ks], axis=0), ts[ks], axis=0)
    pred = np.degrees(np.arctan(np.hypot(acc[..., 0], acc[..., 1]) / G))
    # Two point-mass predictions. The designed pattern (simulation) asks very
    # little once the start-up transient is over; the path the drone ACTUALLY
    # flew is bigger, because it grew, and asks more. Measured tilt above the
    # second number is tilt the point-mass model does not account for.
    print('\n  TILT over the clean window (deg from level)')
    print('    drone     hover   median    p95    max   |  point mass p95/max: '
          'designed pattern   path flown')
    have_tilt = any(f'tilt_{ids[j]}' in cols for j in real)
    for j in real:
        c = f'tilt_{ids[j]}'
        sm = np.stack([np.convolve(P[:, j, ax], np.ones(5) / 5, mode='same')
                       for ax in (0, 1)], 1)
        ap_ = np.gradient(np.gradient(sm, t, axis=0), t, axis=0)[k]
        flown = np.degrees(np.arctan(np.hypot(ap_[:, 0], ap_[:, 1]) / G))
        pr = (f'{np.percentile(pred[:, j], 95):5.1f} / {pred[:, j].max():4.1f}      '
              f'{np.percentile(flown, 95):5.1f} / {flown.max():4.1f}')
        if c not in cols or not np.isfinite(D[k, cols.index(c)]).any():
            print(f'    {ids[j]:8s}    --  (no tilt column in this record)       |  {pr}')
            continue
        tl = D[:, cols.index(c)]
        hov = tl[(t > first[j] + 3) & (t < t_start - 0.5)]
        hov = np.nanmedian(hov) if np.isfinite(hov).any() else float('nan')
        w = tl[k][np.isfinite(tl[k])]
        print(f'    {ids[j]:8s} {hov:6.1f} {np.median(w):8.1f} {np.percentile(w, 95):6.1f} '
              f'{w.max():6.1f}   |  {pr}')
    if not have_tilt:
        print('    (records from before 2026-09-15 carry no tilt; the prediction still stands)')

    # gains
    if virt:
        clamp = a.virtual_clamp if a.virtual_clamp is not None else max_acc
        e1, sc, e = which_gains(t, P, ids, virt, cfg, t_start, t_end, clamp)
        print('\n  GAINS THAT FLEW (virtual drones replayed against the logged real ones)')
        print(f'    config gains as written   {e1 * 100:5.1f} cm RMS')
        print(f'    best fit                  time scale x{sc:.2f}  ({e * 100:.1f} cm RMS)')
        if abs(sc - 1) <= 0.05 and e1 < 0.03:
            print('    -> the config gains flew')
        else:
            print(f'    -> NOT the config gains. The flight matches them sped up x{sc:.2f};'
                  f' a fig4 rung "rescale N" therefore flew as rescale ~{sc:.2f}N')


if __name__ == '__main__':
    main()
