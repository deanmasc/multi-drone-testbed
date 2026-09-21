#!/usr/bin/env python3
"""Shared machinery for the coverage and flocking speed ladders.

The trochoidal ladder (tools/plot_trochoidal_ladder.py) established a method:
replay the control law over the logged positions, cross-correlate the command it
would have asked for against the acceleration the drone actually produced to get
the loop delay tau, band-pass the position to isolate the wobble, and read the
own-velocity gain k out of the law. `k * tau` is then comparable across
algorithms. This module is that method with the trochoidal specifics removed, so
coverage and flocking are measured the same way rather than a similar way.

Two deliberate choices, both worth knowing when reading the numbers:

  * The replayed command is computed from the logged positions with NO delay --
    fresh state, as the paper assumes. So the lag that comes out of the
    cross-correlation is the whole round trip (our sensing lag plus the
    actuation lag), which is the quantity that enters the stability condition.
    See docs/PROJECT_AIM.md section 13d.

  * Velocity is a centred difference of the logged 10 Hz positions (a 0.2 s
    window). The flight stack used a 10-sample fit at 100 Hz (~0.1 s). The
    replay is therefore a slightly smoother version of what flew; it is the
    same choice the trochoidal ladder made, so the three ladders stay
    comparable.
"""

import math
import os
import re
import sys

import numpy as np
import yaml
from scipy.signal import butter, sosfiltfilt, welch

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
CFG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed', 'config')
LOGS = os.path.join(ROOT, 'logs', 'hw')
_PKG = os.path.join(ROOT, 'ros2_ws', 'src', 'drone_testbed')
for p in (_HERE, _PKG):
    if p not in sys.path:
        sys.path.insert(0, p)

import analyse_trochoidal as A                                   # noqa: E402
from drone_testbed.utils.types import DroneState                 # noqa: E402
from drone_testbed.algorithms.registry import get_algorithm      # noqa: E402
import drone_testbed.algorithms                                  # noqa: E402,F401

DT = 0.1                      # the recorder's row interval
WOBBLE_BAND = (0.6, 1.5)      # Hz -- 0.67 to 1.67 s, brackets 4*tau ~ 1.1 s
LAGS = np.arange(-0.4, 0.81, 0.01)
G = 9.81
TAU_NOMINAL = 0.28            # measured round-trip loop delay, s

# ---- ink -------------------------------------------------------------------
# One hue family per algorithm so a figure is identifiable at a glance in the
# combined index; light-to-dark WITHIN a family encodes the rung, because the
# rungs are an ordered sequence (slow to fast), not unrelated categories.
INK, MUTED, GRID, DESIGN = '#1f2328', '#6b7280', '#e5e7eb', '#9ca3af'
SIM = '#9ca3af'
RAMP = {
    # blues, light to dark
    'coverage': ['#8fc0ee', '#4a90d9', '#1b4f8f'],
    # greens, light to dark
    'flocking': ['#8fd3b6', '#2aa877', '#0f5c42'],
}
MARKERS = ['o', 's', '^', 'D']

RC = {
    'font.size': 9, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
    'axes.edgecolor': MUTED, 'axes.labelcolor': INK, 'xtick.color': MUTED,
    'ytick.color': MUTED, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.6, 'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'axes.axisbelow': True, 'savefig.dpi': 170, 'savefig.bbox': 'tight',
}


# ---- reading a record ------------------------------------------------------

def load(path):
    """Columns, data, and the algorithm params the recorder copied into the header."""
    cols, D = A.load_record(path)
    params, ids, in_params = {}, [], False
    with open(path) as fh:
        for line in fh:
            if not line.startswith('#'):
                break
            s = line[1:].strip()
            if s.startswith('drones'):
                ids = [x.strip() for x in s.split(None, 1)[1].split(',')]
            if s.startswith('algorithm params'):
                in_params = True
                continue
            if in_params:
                m = re.match(r'^([a-z_0-9]+):\s*(.+)$', s)
                if not m:
                    if s.startswith('initial positions') or s.startswith('columns'):
                        in_params = False
                    continue
                try:
                    params[m.group(1)] = yaml.safe_load(m.group(2))
                except yaml.YAMLError:
                    params[m.group(1)] = m.group(2)
    return dict(path=path, cols=cols, D=D, t=D[:, 0], params=params, ids=ids)


def positions(rec):
    """(T, n, 2) positions, or None when the record predates position logging."""
    cols, D, ids = rec['cols'], rec['D'], rec['ids']
    if not all(f'x_{i}' in cols for i in ids):
        return None
    return np.stack([np.stack([D[:, cols.index(f'x_{i}')],
                               D[:, cols.index(f'y_{i}')]], 1) for i in ids], 1)


def window(rec, P):
    """The clean window: algorithm start (+2 s settle) to the last live row.

    Virtual drones sit exactly still until the algorithm starts, which is a
    sharper marker of t_start than anything in the header.
    """
    t = rec['t']
    last = A.live_end(P)
    real, virt, grounded, first, t_start = A.classify(t, P, last, rec['ids'])
    if t_start is None:
        t_start = float(t[0])
    k0 = int(np.searchsorted(t, t_start + 2.0))
    return dict(real=real, virt=virt, grounded=grounded, t_start=t_start,
                k0=k0, k1=last, sl=slice(k0, last + 1))


# ---- filters and the wobble ------------------------------------------------

def lp(x, fc, axis=0):
    return sosfiltfilt(butter(4, fc, 'low', fs=1 / DT, output='sos'), x, axis=axis)


def bp(x, lo=WOBBLE_BAND[0], hi=WOBBLE_BAND[1], axis=0):
    return sosfiltfilt(butter(4, [lo, hi], 'band', fs=1 / DT, output='sos'),
                       x, axis=axis)


def wobble(P):
    """Per-drone ripple in the wobble band, split into shared and own motion.

    A formation can oscillate as one body, which every pairwise metric is blind
    to (see FlockingMetrics in tools/metrics_recorder.py). Splitting the fleet
    mean out is what makes that visible.
    """
    n = P.shape[1]
    Z = P[:, :, 0] + 1j * P[:, :, 1]
    ripple = bp(Z.real) + 1j * bp(Z.imag)
    shared = ripple.mean(axis=1)
    out = dict(rms=[], own_rms=[], period=[], circ=[], sense=[],
               shared_rms=float(np.sqrt(np.mean(np.abs(shared) ** 2))))
    for j in range(n):
        r = ripple[:, j]
        out['rms'].append(float(np.sqrt(np.mean(np.abs(r) ** 2))))
        own = r - shared
        out['own_rms'].append(float(np.sqrt(np.mean(np.abs(own) ** 2))))
        z = Z[:, j] - Z[:, j].mean()
        f, psd = welch(z, fs=1 / DT, nperseg=min(256, len(z)),
                       return_onesided=False, detrend='linear')
        band = (np.abs(f) >= WOBBLE_BAND[0]) & (np.abs(f) <= WOBBLE_BAND[1])
        if band.any() and psd[band].sum() > 0:
            pk = f[band][np.argmax(psd[band])]
            ccw = psd[band & (f > 0)].sum() / psd[band].sum()
            out['period'].append(float(1 / abs(pk)))
            out['circ'].append(float(max(ccw, 1 - ccw)))
            out['sense'].append('CCW' if ccw > 0.5 else 'CW')
        else:
            out['period'].append(float('nan'))
            out['circ'].append(float('nan'))
            out['sense'].append('-')
    return out


# ---- replaying the law -----------------------------------------------------

def replay(algo_name, params, ids, P, t_start, n_rows, clamp=None):
    """The acceleration the law would ask for, given the positions that flew.

    Runs the repo's own algorithm class at the flight's 10 Hz so any internal
    clock (the coverage hotspot, the flocking gamma-agent) advances exactly as
    it did in the air. Returns (unclamped, clamped).
    """
    p = dict(params)
    p.pop('log_file', None)
    clamp = float(p.get('max_accel', 0.5)) if clamp is None else clamp
    V = np.gradient(P, DT, axis=0)

    algo = get_algorithm(algo_name)
    algo.configure(dict(p, max_accel=1e9), list(ids))       # unclamped
    U = np.full((n_rows, len(ids), 2), np.nan)
    for k in range(n_rows):
        st = {i: DroneState(i, P[k, j].copy(), V[k, j].copy())
              for j, i in enumerate(ids)}
        out = algo.compute_controls(st, DT)
        for j, i in enumerate(ids):
            U[k, j] = out[i].acceleration
    return U, np.clip(U, -clamp, clamp)


# crazyflie_node, as launched: it integrates the commanded acceleration into a
# position setpoint at 25 Hz, caps the speed, leashes the setpoint to the drone
# and clamps it to the geofence. Same constants as plot_trochoidal_ladder.py.
CF_RATE, CF_MAX_VEL, CF_MAX_LEAD, CF_GEOFENCE = 25.0, 0.7, 0.3, 1.5


def setpoint(t, P, j, Uc, t0, t1):
    # Uc is the CLAMPED command for drone j alone, shape (T, 2).
    """The position setpoint crazyflie_node streamed to drone `j`, rebuilt.

    This is what the drone was actually TOLD to be, which is a different thing
    from both where it was and where the law wanted it. Separating the two is
    what settled the "is the drone shaking?" question on trochoidal: the drone
    tracks its setpoint closely; the setpoint itself is what loops.
    """
    dt = 1.0 / CF_RATE
    taus = np.arange(t0, t1, dt)
    real = np.stack([np.interp(taus, t, P[:, j, ax]) for ax in (0, 1)], 1)
    k = np.clip(np.searchsorted(t, taus, side='right') - 1, 0, len(Uc) - 1)
    pos, vel = real[0].copy(), np.zeros(2)
    out, leashed = np.empty((len(taus), 2)), 0
    for n in range(len(taus)):
        vel = vel + Uc[k[n]] * dt
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
    return taus, out, real, leashed / max(1, len(taus))


def own_velocity_gain(algo_name, params, P, sl):
    """k, the coefficient on a drone's OWN velocity in its commanded accel.

    coverage:  kd
    flocking:  c2_gamma + c2_alpha * sum_j bump_ij, so it depends on how many
               neighbours are in range -- measured over the window, per drone.
    """
    n = P.shape[1]
    if algo_name.lower() == 'coverage':
        k = float(params.get('gain_kd', 1.2))
        return [k] * n
    from drone_testbed.algorithms.flocking import _sigma_norm, _bump
    eps = float(params.get('epsilon', 0.1))
    h = float(params.get('bump_h', 0.2))
    c2a = float(params.get('gain_c2_alpha', 2.0))
    c2g = float(params.get('gain_c2_gamma', 1.4))
    r_alpha = _sigma_norm(np.array([float(params.get('sense_range', 0.9)), 0.0]), eps)
    out = []
    for a in range(n):
        sb = []
        for k in range(*sl.indices(len(P))):
            tot = 0.0
            for b in range(n):
                if a == b:
                    continue
                z = _sigma_norm(P[k, b] - P[k, a], eps)
                tot += _bump(z / r_alpha, h)
            sb.append(tot)
        out.append(c2g + c2a * float(np.mean(sb)))
    return out


def lag(u, acc):
    """Cross-correlation of command against achieved acceleration, over LAGS.

    The peak is the round-trip delay: state -> command -> thrust -> motion.
    """
    n = len(u)
    tg = np.arange(n) * DT
    xc = []
    for L in LAGS:
        ai = np.stack([np.interp(tg + L, tg, acc[:, ax]) for ax in (0, 1)], 1)
        m = (tg + L >= tg[0]) & (tg + L <= tg[-1])
        denom = math.sqrt(float(np.sum(u[m] ** 2) * np.sum(ai[m] ** 2)))
        xc.append(float(np.sum(u[m] * ai[m]) / denom) if denom > 0 else 0.0)
    xc = np.array(xc)
    return LAGS, xc, float(LAGS[np.argmax(xc)]), float(xc.max())


def tilt(acc):
    """Bank angle a point mass needs for this horizontal acceleration, degrees."""
    return np.degrees(np.arctan2(np.hypot(acc[..., 0], acc[..., 1]), G))


# ---- the matching simulation ----------------------------------------------

def sim_of(cfgname, duration):
    """Same config, no physical layer: the gap-B reference (PROJECT_AIM 6a)."""
    import sim_baseline as S
    cfg = yaml.safe_load(open(os.path.join(CFG, cfgname)))
    metrics, ts, data, pos_hist, rate = S.run(cfg, duration)
    return dict(cfg=cfg, cols=metrics.all_columns(), t=ts, D=data,
                P=np.array(pos_hist), ids=[d['id'] for d in cfg['drones']])


def recompute_coverage_H(rec, P, t_start):
    """Coverage cost on the ALGORITHM's clock rather than the recorder's.

    CoverageMetrics.row() integrates the density at the time it is handed, and
    the recorder hands it its own t. That is harmless for a static density --
    every run before 2026-09-16 -- but the speed ladder moves the hotspot at
    0.3 rad/s, so a 12.5 s difference between the two clocks scores the fleet
    against a hotspot most of an orbit away. Everything else in the record is
    unaffected: positions, cdist geometry and the wobble never used t.
    """
    import metrics_recorder as M
    cm = M.CoverageMetrics(rec['ids'], rec['params'])
    t = rec['t']
    H = np.full(len(t), np.nan)
    for k in range(len(t)):
        cells = cm._cells(P[k], t[k] - t_start)
        H[k] = sum(c[1] for c in cells if c is not None)
    return H


def col(rec_or_sim, name):
    cols = rec_or_sim['cols']
    return rec_or_sim['D'][:, cols.index(name)] if name in cols else None
