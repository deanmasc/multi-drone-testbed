#!/usr/bin/env python3
"""Record the metric that actually tests each algorithm's theorem.

A theorem promises a *property*, not a trajectory. Olfati-Saber never promises
a path -- it promises cohesion, no collisions, and matched velocities. So path
error tests something no paper claimed, and the quantity worth recording is
different for every algorithm. See docs/PROJECT_AIM.md section 7.

  Flocking    pairwise spacing against the target lattice, the smallest gap
              ever reached (the collision-avoidance promise), velocity spread
              across the fleet, and whether the graph stayed connected.

  Coverage    the locational cost H(p) over time, each agent's distance from
              its own cell centroid, and -- at exit -- H against the value an
              offline Lloyd iteration converges to from the same start. That
              ratio is the "what optimality did hardware achieve" number.

  DistanceFormation
              per-edge distance error, and -- separately -- whether the shape
              that satisfied those edges was the one asked for. Those come
              apart: on a merely rigid graph a folded corner satisfies every
              constraint exactly. Also the Lyapunov function, which the
              continuous law forbids from rising, and the fleet centroid,
              which the continuous law forbids from moving.

  Trochoidal  the two frequencies, their ratio, and the two radii, recovered
              at exit by FFT of the complex signal z = x + iy (a trochoid is a
              sum of two complex exponentials, so the spectrum has exactly two
              peaks). Plus the decay time constant of the envelope, which is
              the number the control-rate sweep needs: a trochoid only exists
              while its poles sit on the imaginary axis, and anything that
              moves them off shows up here as a finite tau.

Rows are streamed to disk as they are computed, not buffered to the end. This
testbed aborts mid-flight often enough that a recorder which only writes on
clean shutdown would lose exactly the runs worth studying. The derived summary
is appended when the node stops, however it stops.

Output goes to <out-dir>/<algorithm>_<YYYYmmdd_HHMMSS>.txt, loadable with
numpy.loadtxt (the header and summary are '#'-commented).

Usage -- run in its own terminal alongside the normal two:

    python3 tools/metrics_recorder.py --config ros2_ws/src/drone_testbed/config/testbed_flocking.yaml

    python3 tools/metrics_recorder.py \
        --config ros2_ws/src/drone_testbed/config/testbed_coverage_hybrid.yaml \
        --out-dir ~/flights

Stop it with Ctrl-C when the flight ends; that is what triggers the analysis.
"""

import argparse
import math
import os
import sys
from datetime import datetime
from itertools import combinations

import numpy as np
import yaml

# Coverage's analysis borrows the algorithm's own Voronoi code, which lives in
# the package. Off the lab PC nothing has sourced the workspace, so --analyse
# would fall back to "H not computed" on exactly the machine the re-analysis is
# meant to happen on. Point at the package directly, as sim_baseline.py does.
_PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'ros2_ws', 'src', 'drone_testbed')
if os.path.isdir(_PKG) and _PKG not in sys.path:
    sys.path.insert(0, _PKG)

# ROS is only needed to *record*. Guarding the import keeps the metric classes
# below importable on a machine without ROS, so a finished record file can be
# re-analysed anywhere -- including the laptop, away from the lab.
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String
    HAVE_ROS = True
except ImportError:                         # pragma: no cover
    HAVE_ROS = False
    Node = object


# Speed below which the fleet counts as not yet flying. Used to find the start
# of real motion, so the auto_start_delay's worth of stationary rows does not
# end up in the trochoidal spectrum as a DC-heavy leading edge.
#
# 0.02 m/s was too low: VICON noise differentiated into velocity puts brief
# spikes above that on drones sitting still on the floor, and the first hardware
# flight consequently "detected motion" at t=0.8s during a 12s pre-flight hold.
# Require a larger speed, sustained, before believing it.
MOTION_SPEED = 0.06      # m/s
MOTION_HOLD = 0.7        # s the fleet must keep moving before the run counts

# A pair closer than this is a genuine near-miss worth flagging in the summary.
NEAR_MISS = 0.25         # m


# ---------------------------------------------------------------------------
# metric sets
# ---------------------------------------------------------------------------

class MetricSet:
    """Per-algorithm columns and end-of-run analysis.

    columns() names what a row holds; row() computes one; summarise() gets the
    whole recorded array back and returns lines of text.
    """

    def __init__(self, ids, params):
        self.ids = ids
        self.params = params
        self.pairs = list(combinations(range(len(ids)), 2))

    def columns(self):
        raise NotImplementedError

    def row(self, t, pos, vel):
        raise NotImplementedError

    def summarise(self, t, data, pos_hist):
        return []


class FlockingMetrics(MetricSet):
    """Cohesion, collision avoidance, velocity matching -- Olfati-Saber's three."""

    def __init__(self, ids, params):
        super().__init__(ids, params)
        self.d = float(params.get('spacing', 0.6))
        self.r = float(params.get('sense_range', 0.9))

    def columns(self):
        cols = ['t']
        cols += [f'd_{self.ids[a]}_{self.ids[b]}' for a, b in self.pairs]
        cols += ['d_min', 'd_mean', 'lattice_err', 'vel_spread',
                 'n_edges', 'connected']
        # Per-drone position, appended so every existing column keeps its index.
        #
        # Everything above is a *pairwise* quantity, and pairwise metrics are
        # blind to motion the whole fleet shares: if two drones trace the same
        # small circle in phase, their separation never changes and the wobble
        # is invisible. The first hardware flocking run was read that way and
        # the conclusion was wrong -- the operators could see all four drones
        # circling on the live plot while the metrics reported a quiet fleet.
        # Positions are the only thing that cannot hide a common mode, and
        # they let reanalyse() recompute anything later.
        cols += [f'{ax}_{i}' for i in self.ids for ax in ('x', 'y')]
        return cols

    def row(self, t, pos, vel):
        dists = np.array([np.linalg.norm(pos[a] - pos[b])
                          for a, b in self.pairs])

        # Lattice error is only meaningful for pairs that can actually see each
        # other. Two drones on opposite sides of the flock are not trying to sit
        # at d apart, so folding them in would report a fault that is not one.
        near = dists <= self.r
        if near.any():
            lattice_err = float(np.sqrt(np.mean((dists[near] - self.d) ** 2)))
        else:
            lattice_err = float('nan')

        # Velocity matching: RMS distance of each velocity from the fleet mean.
        # Goes to zero when the flock moves as one body, whatever that body's
        # own speed is.
        vel_spread = float(np.sqrt(np.mean(
            ((vel - vel.mean(axis=0)) ** 2).sum(axis=1))))

        return ([t] + list(dists) +
                [float(dists.min()), float(dists.mean()), lattice_err,
                 vel_spread, float(near.sum()),
                 float(self._connected(near))] +
                [float(v) for v in np.asarray(pos).reshape(-1)])

    def _connected(self, near):
        """Union-find over the edges that exist this tick."""
        n = len(self.ids)
        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for (a, b), present in zip(self.pairs, near):
            if present:
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[ra] = rb
        return len({find(i) for i in range(n)}) == 1

    # -- wobble --------------------------------------------------------------
    #
    # The flock rides a slow orbit set by the gamma leader (tens of seconds per
    # lap) and, on hardware, carries a much faster ripple on top of it. The two
    # separate cleanly by scale: subtract a centred moving average long enough
    # to erase the orbit and short enough to leave the ripple untouched.

    WOBBLE_BAND = (0.3, 6.0)     # s -- ripple periods worth reporting
    WOBBLE_TRIM = 0.10           # fraction of each end to discard
    WOBBLE_SOFT = 0.6            # width of the filter's edge ramps

    @staticmethod
    def _bandpass(t, y, lo, hi, soft=WOBBLE_SOFT):
        """Keep only periods between lo and hi seconds.

        Two traps here, both of which produced a convincing phantom wobble
        before they were fixed:

        Subtracting a short moving average is the obvious way to strip the slow
        orbit, and it is not sharp enough -- a 3 s boxcar still passes ~2% of a
        25 s orbit, which on a 0.6 m orbit radius is 1.4 cm of fake ripple, the
        same size as the real thing.

        Replacing it with a brick-wall band still left 0.4 cm, because a sharp
        spectral cut rings at the ends of the record. Ramping the filter edges
        smoothly instead drops that to 0.01 cm while recovering a known 3.00 cm
        circle as 3.00 cm. Trim the ends afterwards regardless.
        """
        n = len(y)
        y = y - np.polyval(np.polyfit(t, y, 1), t)   # endpoints matter to a DFT
        Y = np.fft.rfft(y)
        f = np.fft.rfftfreq(n, float(np.median(np.diff(t))))
        f_lo, f_hi = 1.0 / hi, 1.0 / lo
        if soft <= 0:
            gain = ((f >= f_lo) & (f <= f_hi)).astype(float)
        else:
            gain = np.ones_like(f)
            a, b = f_lo / (1 + soft), f_lo * (1 + soft)
            m = (f > a) & (f < b)
            gain[f <= a] = 0.0
            gain[m] = 0.5 * (1 - np.cos(np.pi * (f[m] - a) / (b - a)))
            a, b = f_hi / (1 + soft), f_hi * (1 + soft)
            m = (f > a) & (f < b)
            gain[m] = 0.5 * (1 + np.cos(np.pi * (f[m] - a) / (b - a)))
            gain[f >= b] = 0.0
        return np.fft.irfft(Y * gain, n)

    @classmethod
    def _ellipse(cls, t, wx, wy):
        """Describe the dominant ripple as a rotating ellipse.

        Any planar oscillation is the sum of two counter-rotating circles, and
        the FFT of the complex signal x+iy separates them by the sign of the
        frequency. Their sum is the ellipse's major axis and their difference
        its minor, so |A+ - A-| / (A+ + A-) is 1 for a clean circle and 0 for a
        straight line -- which is precisely the question worth asking here: are
        the drones tracing little circles, or shuffling back and forth?
        """
        n = len(wx)
        if n < 32:
            return None
        dt = float(np.median(np.diff(t)))
        w = np.hanning(n)
        Z = np.fft.fftshift(np.fft.fft((wx + 1j * wy) * w, 8 * n))
        f = np.fft.fftshift(np.fft.fftfreq(8 * n, dt))
        mag = np.abs(Z) / (w.sum() / 2)
        lo, hi = cls.WOBBLE_BAND
        mag[(np.abs(f) < 1.0 / hi) | (np.abs(f) > 1.0 / lo)] = 0.0
        k = int(np.argmax(mag))
        if mag[k] <= 0.0 or f[k] == 0.0:
            return None
        big, small = mag[k], mag[int(np.argmin(np.abs(f + f[k])))]
        major, minor = big + small, abs(big - small)
        return {'period': 1.0 / abs(f[k]), 'major': major,
                'circularity': minor / major if major > 0 else 0.0,
                'sense': 'CCW' if f[k] > 0 else 'CW'}

    def _wobble_lines(self, t, data, c):
        """Per-drone ripple, split into what the fleet shares and what it does
        not. A pairwise metric cannot see the shared part at all."""
        if any(f'x_{i}' not in c for i in self.ids) or len(t) < 64:
            return []
        lo, hi = self.WOBBLE_BAND
        cut = max(1, int(self.WOBBLE_TRIM * len(t)))
        sl = slice(cut, len(t) - cut)
        tt = t[sl]

        xy = np.stack([data[:, [c[f'x_{i}'], c[f'y_{i}']]] for i in self.ids])
        if not np.isfinite(xy).all():       # an older record, padded with NaN
            return []
        band = lambda v: np.stack(                              # noqa: E731
            [self._bandpass(t, v[:, k], lo, hi)[sl] for k in (0, 1)], axis=1)
        common = band(xy.mean(axis=0))

        out = ['', f'  WOBBLE (ripple with a period between '
                   f'{lo:g} s and {hi:g} s)',
               '    drone            ripple    own    period   shape']
        for j, i in enumerate(self.ids):
            w = band(xy[j])
            e = self._ellipse(tt, w[:, 0], w[:, 1])
            own = w - common
            r_tot = float(np.sqrt((w ** 2).sum(axis=1).mean()))
            r_own = float(np.sqrt((own ** 2).sum(axis=1).mean()))
            if e is None:
                out.append(f'    {i:12s} {r_tot*100:7.2f}cm {r_own*100:6.2f}cm'
                           f'        --   --')
                continue
            shape = ('circle' if e['circularity'] > 0.6 else
                     'ellipse' if e['circularity'] > 0.25 else 'line')
            out.append(f'    {i:12s} {r_tot*100:7.2f}cm {r_own*100:6.2f}cm '
                       f'{e["period"]:7.2f}s   {shape} {e["sense"]} '
                       f'(e={e["circularity"]:.2f})')

        rc = float(np.sqrt((common ** 2).sum(axis=1).mean()))
        ec = self._ellipse(tt, common[:, 0], common[:, 1])
        out.append(f'    {"fleet together":12s} {rc*100:7.2f}cm'
                   + (f'          {ec["period"]:7.2f}s   '
                      f'e={ec["circularity"]:.2f}' if ec else ''))
        out.append('')
        out.append('    "ripple" is each drone\'s own motion, "own" is what is')
        out.append('    left once the shared fleet-wide ripple is subtracted.')
        out.append('    A large fleet-together figure means the whole formation')
        out.append('    is oscillating as one body -- which the pair distances')
        out.append('    and the velocity spread are both blind to.')
        return out

    def summarise(self, t, data, pos_hist):
        c = {name: i for i, name in enumerate(self.columns())}
        d_min = data[:, c['d_min']]
        worst = int(np.argmin(d_min))
        connected = data[:, c['connected']]
        tail = slice(max(0, len(t) - int(0.1 * len(t)) - 1), None)

        lat = data[:, c['lattice_err']]
        out = [
            'FLOCKING -- did the theorem\'s promises hold?',
            '',
            f'  target spacing d            {self.d:.3f} m',
            f'  sense range r               {self.r:.3f} m',
            '',
            '  COLLISION AVOIDANCE',
            f'    smallest gap ever         {d_min[worst]:.4f} m  at t = {t[worst]:.1f} s',
            f'    ticks under {NEAR_MISS:.2f} m         {int((d_min < NEAR_MISS).sum())} of {len(t)}',
            '',
            '  COHESION (final 10% of run)',
            f'    lattice error             {np.nanmean(data[tail, c["lattice_err"]]):.4f} m rms',
            f'    mean pair separation      {np.nanmean(data[tail, c["d_mean"]]):.4f} m',
            '',
            '  VELOCITY MATCHING (final 10% of run)',
            f'    velocity spread           {np.nanmean(data[tail, c["vel_spread"]]):.4f} m/s',
            f'    peak during run           {np.nanmax(data[:, c["vel_spread"]]):.4f} m/s',
            '',
            '  CONNECTIVITY',
            f'    connected for             {100.0 * connected.mean():.1f}% of ticks',
        ]
        # Starting scattered and out of range is normal and is not a split. Only
        # a break *after* the flock has once been whole is a loss of cohesion.
        first_conn = np.where(connected > 0)[0]
        if len(first_conn) == 0:
            out.append('    never became connected')
        else:
            f0 = int(first_conn[0])
            if f0 > 0:
                out.append(f'    became connected at       t = {t[f0]:.1f} s '
                           f'(started scattered)')
            after = connected[f0:]
            if after.min() < 1:
                split = f0 + int(np.argmin(after))
                out.append(f'    SPLIT after forming at    t = {t[split]:.1f} s')
            else:
                out.append('    stayed connected once formed')

        # Settling is measured against where the lattice actually converges, not
        # against d. The gamma (leader) term squeezes the group inside the target
        # spacing by a fixed factor, so a tolerance around d would report a
        # permanent failure for behaviour that is correct and expected.
        final = float(np.nanmean(lat[tail]))
        band = max(0.05 * self.d, 0.1 * abs(final))
        bad = np.where(np.abs(lat - final) > band)[0]
        out.append('')
        out.append(f'  CONVERGENCE (lattice error settles at {final:.4f} m, '
                   f'not 0 -- the gamma')
        out.append('               term compresses the lattice below d by design)')
        if len(bad) == 0:
            out.append(f'    already settled at the first sample')
        elif bad[-1] < len(t) - 1:
            out.append(f'    settled to within {band:.3f} m at t = {t[bad[-1] + 1]:.1f} s')
        else:
            out.append(f'    still moving at end of run (never settled)')
        out.append(f'    final mean spacing / d    {np.nanmean(data[tail, c["d_mean"]]) / self.d:.3f}')
        out += self._wobble_lines(t, data, c)
        return out


class CoverageMetrics(MetricSet):
    """Locational cost H and centroid distances, plus the optimality ratio."""

    def __init__(self, ids, params):
        super().__init__(ids, params)
        self._algo = None
        try:
            from drone_testbed.algorithms.coverage import Coverage
            self._algo = Coverage()
            self._algo.configure(dict(params), list(ids))
        except Exception as exc:            # noqa: BLE001 -- reported, not raised
            print(f'[metrics] coverage helpers unavailable ({exc});\n'
                  f'          H will not be computed. Source the ROS 2 workspace '
                  f'so drone_testbed is importable.', file=sys.stderr)

    def available(self):
        return self._algo is not None

    def columns(self):
        cols = ['t', 'H']
        cols += [f'cdist_{i}' for i in self.ids]
        cols += ['cdist_max', 'cdist_mean']
        return cols

    def _cells(self, pos, t):
        """Voronoi cell, centroid and cost contribution for every agent."""
        a = self._algo
        out = []
        for k in range(len(pos)):
            poly = a._square.copy()
            for j in range(len(pos)):
                if j == k:
                    continue
                d = pos[j] - pos[k]
                if float(np.dot(d, d)) < 1e-12:
                    continue
                from drone_testbed.algorithms.coverage import _clip_halfplane
                poly = _clip_halfplane(poly, 0.5 * (pos[k] + pos[j]), d)
                if poly is None:
                    break
            if poly is None:
                out.append(None)
                continue
            out.append(a._cell_integral(poly, pos[k], t))
        return out

    def row(self, t, pos, vel):
        if self._algo is None:
            return None
        cells = self._cells(pos, t)
        H = 0.0
        cdist = []
        for k, res in enumerate(cells):
            if res is None:
                cdist.append(float('nan'))
                continue
            centroid, cost = res
            H += cost
            cdist.append(float(np.linalg.norm(centroid - pos[k])))
        cd = np.array(cdist)
        return ([t, H] + cdist +
                [float(np.nanmax(cd)), float(np.nanmean(cd))])

    def _lloyd_optimum(self, start, iters=400):
        """Run Lloyd's iteration offline, as fast as geometry allows.

        This is the same algorithm with the vehicle removed: jump straight to
        the centroid each step instead of accelerating toward it. It converges
        to the centroidal Voronoi configuration in the same basin as the flight
        started in, which is the right target to score the flight against --
        the global optimum would be a different and unfair comparison, since
        the control law is only ever claimed to find a local minimum.
        """
        pos = np.array(start, dtype=float)
        for _ in range(iters):
            cells = self._cells(pos, 0.0)
            nxt = pos.copy()
            for k, res in enumerate(cells):
                if res is not None:
                    nxt[k] = res[0]
            if np.max(np.abs(nxt - pos)) < 1e-9:
                pos = nxt
                break
            pos = nxt
        cells = self._cells(pos, 0.0)
        H = sum(c[1] for c in cells if c is not None)
        return pos, float(H)

    def summarise(self, t, data, pos_hist):
        if self._algo is None:
            return ['COVERAGE -- H not computed (drone_testbed not importable)']
        c = {name: i for i, name in enumerate(self.columns())}
        H = data[:, c['H']]

        # A gradient descent should never go uphill. Once settled, though, H
        # jitters in the last few decimal places forever, and counting those as
        # violations reports a 40% failure rate for a run that converged
        # perfectly. Only a rise worth 0.1% of the total descent counts.
        dH = np.diff(H)
        floor = 1e-3 * abs(H[0] - H.min()) if H[0] > H.min() else 1e-12
        rises = int((dH > floor).sum())
        worst_rise = float(dH.max()) if len(dH) else 0.0

        out = [
            'COVERAGE -- did it reach a centroidal Voronoi configuration?',
            '',
            '  LOCATIONAL COST H',
            f'    initial                   {H[0]:.6f}',
            f'    final                     {H[-1]:.6f}',
            f'    minimum reached           {H.min():.6f}  at t = {t[int(np.argmin(H))]:.1f} s',
            f'    reduction                 {100.0 * (H[0] - H[-1]) / H[0]:.2f}%',
            f'    ticks where H rose        {rises} of {len(H) - 1}  '
            f'(rises above {floor:.2e})',
            f'    largest single rise       {worst_rise:+.3e}',
            '',
            '  CENTROID CONVERGENCE (each agent should sit on its own centroid)',
            f'    final max distance        {data[-1, c["cdist_max"]]:.4f} m',
            f'    final mean distance       {data[-1, c["cdist_mean"]]:.4f} m',
        ]

        try:
            opt_pos, H_opt = self._lloyd_optimum(pos_hist[0])
            out += [
                '',
                '  OPTIMALITY  (vs offline Lloyd from the same starting positions)',
                f'    H at Lloyd convergence    {H_opt:.6f}',
                f'    H achieved in flight      {H[-1]:.6f}',
                f'    ratio H_flight / H_lloyd  {H[-1] / H_opt:.4f}',
                f'    excess cost               {100.0 * (H[-1] - H_opt) / H_opt:+.2f}%',
                '',
                '    Lloyd final positions:',
            ]
            for i, p in zip(self.ids, opt_pos):
                out.append(f'      {i:<10s} ({p[0]:+.4f}, {p[1]:+.4f})')
            # Under --analyse only the starting marks are recoverable (the
            # recorded columns hold centroid distances, not positions), so this
            # listing is omitted rather than printed with the wrong numbers.
            if len(pos_hist) > 1:
                out.append('    flight final positions:')
                for i, p in zip(self.ids, pos_hist[-1]):
                    out.append(f'      {i:<10s} ({p[0]:+.4f}, {p[1]:+.4f})')
            out += [
                '',
                f'    NOTE grid_res = {self._algo._grid}. The density integral is'
                ' discretised, so',
                '         some of any residual gap is ours, not the hardware\'s.',
            ]
        except Exception as exc:            # noqa: BLE001
            out.append(f'\n  optimality comparison failed: {exc}')
        return out


class TrochoidalMetrics(MetricSet):
    """Frequencies, ratio, radii and envelope decay -- recovered at exit."""

    def columns(self):
        cols = ['t']
        for i in self.ids:
            cols += [f'x_{i}', f'y_{i}', f'r_{i}', f'speed_{i}']
        return cols

    def row(self, t, pos, vel):
        # The pattern centre is not known until the run is over, so radius here
        # is measured from the origin -- the alpha term centres the pattern
        # there. The summary recomputes it about the true time-mean.
        out = [t]
        for k in range(len(pos)):
            out += [float(pos[k, 0]), float(pos[k, 1]),
                    float(np.linalg.norm(pos[k])),
                    float(np.linalg.norm(vel[k]))]
        return out

    def _spectrum(self, t, xy):
        """Two dominant frequencies and their amplitudes for one agent.

        FFT of the complex signal z = x + iy rather than of x and y separately.
        A trochoid is A1*exp(i*w1*t) + A2*exp(i*w2*t), so the complex spectrum
        has exactly two peaks and their sign carries the direction of rotation
        -- information a pair of real FFTs throws away.
        """
        n = len(t)
        if n < 32:
            return None
        # Resample onto an exactly uniform grid; ROS timer jitter would
        # otherwise smear the peaks.
        fs = (n - 1) / (t[-1] - t[0])
        tu = np.linspace(t[0], t[-1], n)
        zx = np.interp(tu, t, xy[:, 0])
        zy = np.interp(tu, t, xy[:, 1])
        z = (zx - zx.mean()) + 1j * (zy - zy.mean())

        # Zero-pad before transforming. A 205 s trochoidal flight holds only
        # about three cycles of the slow mode, so the raw bin spacing (1/205 Hz)
        # is a large fraction of the frequency being measured. Padding does not
        # add resolution -- the two modes are far apart, so that is not the
        # problem -- but it does let the peak be located between raw bins, which
        # is worth several percent on both frequency and radius.
        pad = 8
        n_fft = n * pad
        w = np.hanning(n)
        Z = np.fft.fft(z * w, n_fft)
        freqs = np.fft.fftfreq(n_fft, 1.0 / fs)
        mag = np.abs(Z) / (n * w.mean())    # window gain correction
        df = fs / n_fft

        order = np.argsort(mag)[::-1]
        picked = []
        for idx in order:
            if freqs[idx] == 0.0:
                continue
            # Reject anything inside a neighbouring peak's skirt: that is the
            # same mode's window sidelobe, not a second one. A Hann main lobe is
            # 4 raw bins wide, so the guard scales with the pad factor.
            if any(abs(idx - p) <= 4 * pad for p in picked):
                continue
            picked.append(idx)
            if len(picked) == 2:
                break
        if len(picked) < 2:
            return None

        out = []
        for k in picked:
            # Parabolic interpolation through the peak and its two neighbours,
            # which recovers the true peak position and height sub-bin.
            m0 = mag[(k - 1) % n_fft]
            m1 = mag[k]
            m2 = mag[(k + 1) % n_fft]
            denom = m0 - 2.0 * m1 + m2
            delta = 0.5 * (m0 - m2) / denom if abs(denom) > 1e-18 else 0.0
            delta = float(np.clip(delta, -0.5, 0.5))
            out.append((float(freqs[k] + delta * df),
                        float(m1 - 0.25 * (m0 - m2) * delta)))

        (f1, a1), (f2, a2) = out
        # Slow mode first, so the ratio is always >= 1 and comparable run to run.
        if abs(f1) > abs(f2):
            (f1, a1), (f2, a2) = (f2, a2), (f1, a1)
        return f1, a1, f2, a2

    @staticmethod
    def _decay_correction(t, tau):
        """Factor by which envelope decay depresses the FFT amplitude.

        The transform reports the *average* amplitude over the window, so a
        pattern that shrinks during the flight reads far smaller than the radius
        it actually had -- about half, for a decay comparable to the run length.
        Dividing by this factor converts the measured amplitude back to the
        radius at the start of the analysis window, which is the quantity the
        paper's initial-condition prediction is about.
        """
        if tau is None or not np.isfinite(tau) or abs(tau) < 1e-9:
            return 1.0
        w = np.hanning(len(t))
        env = np.exp(-(t - t[0]) / tau)
        return float((w * env).sum() / w.sum())

    def _decay(self, t, xy, centre):
        """Envelope time constant, from a straight-line fit to log(peak radius).

        Positive tau means the pattern is shrinking; the trochoid condition
        wants tau = infinity. A finite value here is the number the control-rate
        sweep compares across 10 / 25 / 50 Hz.
        """
        r = np.linalg.norm(xy - centre, axis=1)
        n_win = max(8, len(t) // 12)
        peaks, times = [], []
        for s in range(0, len(t) - n_win, n_win):
            seg = r[s:s + n_win]
            peaks.append(seg.max())
            times.append(t[s:s + n_win].mean())
        peaks = np.asarray(peaks)
        times = np.asarray(times)
        good = peaks > 1e-6
        if good.sum() < 3:
            return None
        slope, _ = np.polyfit(times[good], np.log(peaks[good]), 1)
        if abs(slope) < 1e-9:
            return float('inf')
        return -1.0 / slope

    def summarise(self, t, data, pos_hist):
        c = {name: i for i, name in enumerate(self.columns())}
        out = [
            'TROCHOIDAL -- is the trajectory actually a trochoid?',
            '',
            '  There is no reference trajectory here, so tracking error is',
            '  undefined. The pattern is validated against the two frequencies',
            '  and two radii the eigenvalues predict.',
            '',
        ]
        ratios = []
        for k, drone in enumerate(self.ids):
            xy = np.column_stack([data[:, c[f'x_{drone}']],
                                  data[:, c[f'y_{drone}']]])
            centre = xy.mean(axis=0)
            out.append(f'  {drone}')
            out.append(f'    pattern centre            ({centre[0]:+.4f}, {centre[1]:+.4f})')

            tau = self._decay(t, xy, centre)
            duration = t[-1] - t[0]
            # A time constant many times the run length is not a measurement of
            # decay, it is the fit picking up noise. Do not dignify it with a
            # number, and do not correct amplitudes by it.
            significant = (tau is not None and np.isfinite(tau)
                           and abs(tau) < 10.0 * duration)

            spec = self._spectrum(t, xy)
            if spec is None:
                out.append('    spectrum                  too few samples')
            else:
                f1, a1, f2, a2 = spec
                ratio = abs(f2 / f1) if f1 else float('nan')
                ratios.append(ratio)
                corr = self._decay_correction(t, tau) if significant else 1.0
                out += [
                    f'    slow mode                 {abs(f1):.5f} Hz  '
                    f'(period {1.0 / abs(f1):.2f} s)',
                    f'    fast mode                 {abs(f2):.5f} Hz  '
                    f'(period {1.0 / abs(f2):.2f} s)',
                    f'    period ratio              {ratio:.4f}',
                    f'    rotation senses           slow {"CCW" if f1 > 0 else "CW"}, '
                    f'fast {"CCW" if f2 > 0 else "CW"}',
                    f'    radii, run-average        {a1:.4f} m (slow), '
                    f'{a2:.4f} m (fast)',
                ]
                if corr < 0.999:
                    out.append(
                        f'    radii at analysis start   {a1 / corr:.4f} m (slow), '
                        f'{a2 / corr:.4f} m (fast)   <- compare against the'
                        ' predicted envelope')

            if tau is None:
                out.append('    envelope decay            not enough data')
            elif not significant:
                out.append('    envelope decay            none significant over '
                           f'{duration:.0f} s')
            elif tau > 0:
                out.append(f'    envelope decay tau        {tau:.1f} s  (shrinking; '
                           f'{duration / tau:.2f} time constants this run)')
            else:
                out.append(f'    envelope GROWTH tau       {-tau:.1f} s  (diverging)')
            out.append('')

        if ratios:
            out += [
                f'  period ratio across fleet   mean {np.mean(ratios):.4f}, '
                f'spread {np.std(ratios):.4f}',
                '',
                '  Compare the ratio against the eigenvalue prediction for this',
                '  config (3.3337 for testbed_fig4.yaml). A ratio that matches',
                '  while the envelope decays means the shape is right and only',
                '  the marginal-stability condition failed -- which is the',
                '  expected hardware result. Feed tau into the control-rate',
                '  sweep (docs/PROJECT_AIM.md section 9, experiment 4).',
            ]
        return out


class DistanceFormationMetrics(MetricSet):
    """Did the shape converge -- and was it the RIGHT shape?

    Distance-based formation control is the one algorithm here where the
    obvious metric is actively misleading. Edge error going to zero does not
    mean the hexagon formed: an incorrect equilibrium (a folded corner on a
    merely rigid graph) satisfies every edge constraint exactly and reads as a
    perfect score. So the edge errors are recorded, but the verdict comes from
    fitting the flown positions onto the nominal shape and reporting what is
    left over.

    Four separate promises get scored, because they fail independently:

      edge error    the constraint the law actually minimises
      shape error   whether the constraints were satisfied by the intended
                    realisation or by a different one
      Lyapunov      W = (kp/4) sum e_ij^2 + (kv-weighted anchor term)
                    + (1/2) sum ||v_i||^2 must be non-increasing. Every rise is
                    energy the discrete, saturated implementation injected that
                    the continuous law could not.
      centroid      the interaction forces cancel in pairs, so from rest the
                    fleet centroid cannot move. Drift is a direct measure of
                    the acceleration clamp truncating one agent and not its
                    partner (in simulation) plus everything physical (in the
                    lab).

    Orientation is recorded but is NOT scored. Distances cannot fix an angle,
    so the final heading is set by the initial conditions and any value is
    correct. It is here because "the hexagon came out rotated" is the first
    thing an observer will report as a fault, and the record should be able to
    say plainly that it is not one.
    """

    SETTLE_TOL = 0.01        # m of edge rms below which the shape counts as made
    SHAPE_TOL = 0.05         # m of residual that separates "right shape" from
                             # "converged to something else"

    def __init__(self, ids, params):
        super().__init__(ids, params)
        self.ok = False
        try:
            from drone_testbed.algorithms.distance_formation import (
                formation_spec, rigidity_report)
        except ImportError:
            # No workspace on this machine. Fall back to the pairwise columns
            # so the record is still readable, rather than refusing to run.
            self.targets = None
            self.edges = []
            self.dist = np.zeros(0)
            self.rigidity = None
            return

        self.targets, self.edges, self.dist = formation_spec(params, ids)
        self.rigidity = rigidity_report(self.targets, self.edges)
        self.kp = float(params.get('gain_kp', 0.6))
        self.kv = float(params.get('gain_kv', 1.0))
        self.anchor_kp = float(params.get('anchor_gain_kp', 0.4))
        anchor_ids = [str(a) for a in (params.get('anchor') or [])]
        self.anchors = {ids.index(a): self.targets[ids.index(a)]
                        for a in anchor_ids if a in ids}
        self.ok = True

    # -- geometry ----------------------------------------------------------

    @staticmethod
    def _rigid_fit(pos, target):
        """Best rigid fit of the nominal shape onto the flown one.

        Returns (residual rms, mirrored, rotation in degrees). Both handedness
        options are tried because a set of distances cannot distinguish a shape
        from its mirror image -- no graph can rule that out -- so a mirrored
        formation is a correct outcome that must not be scored as a large
        error. It is reported as a flag instead.
        """
        A = np.asarray(pos, float)
        B = np.asarray(target, float)
        A = A - A.mean(axis=0)
        B = B - B.mean(axis=0)
        best = None
        for mirror in (1.0, -1.0):
            Bm = B * np.array([1.0, mirror])
            U, _, Vt = np.linalg.svd(Bm.T @ A)
            R = U @ np.diag([1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt
            err = float(np.sqrt(((Bm @ R - A) ** 2).sum(axis=1).mean()))
            ang = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            if best is None or err < best[0]:
                best = (err, mirror < 0, ang)
        return best

    def columns(self):
        cols = ['t']
        cols += [f'e_{self.ids[a]}_{self.ids[b]}' for a, b in self.edges]
        cols += ['edge_rms', 'edge_max', 'd_min', 'V_pot', 'K_kin', 'W_lyap',
                 'shape_err', 'mirrored', 'orient_deg', 'cx', 'cy']
        cols += [f'{ax}_{i}' for i in self.ids for ax in ('x', 'y')]
        return cols

    def row(self, t, pos, vel):
        pos = np.asarray(pos, float)
        vel = np.asarray(vel, float)

        errs = np.array([np.linalg.norm(pos[a] - pos[b]) - d
                         for (a, b), d in zip(self.edges, self.dist)])
        # The potential is written in the squared-distance error the law
        # actually descends, not in the length error above -- they are not the
        # same function and only the first is the Lyapunov candidate.
        sq = np.array([float((pos[a] - pos[b]) @ (pos[a] - pos[b])) - d * d
                       for (a, b), d in zip(self.edges, self.dist)])
        V = 0.25 * self.kp * float((sq ** 2).sum())
        for k, target in self.anchors.items():
            V += 0.5 * self.anchor_kp * float((pos[k] - target) @ (pos[k] - target))
        K = 0.5 * float((vel ** 2).sum())

        gaps = [np.linalg.norm(pos[a] - pos[b]) for a, b in self.pairs]
        shape_err, mirrored, orient = self._rigid_fit(pos, self.targets)
        centroid = pos.mean(axis=0)

        return ([t] + [abs(float(e)) for e in errs] +
                [float(np.sqrt((errs ** 2).mean())), float(np.abs(errs).max()),
                 float(min(gaps)), V, K, V + K,
                 shape_err, float(mirrored), orient,
                 float(centroid[0]), float(centroid[1])] +
                [float(v) for v in pos.reshape(-1)])

    # -- analysis ----------------------------------------------------------

    def _rigidity_lines(self):
        r = self.rigidity
        kind = ('minimally rigid' if r['rigid'] and r['minimal']
                else 'rigid' if r['rigid'] else 'FLEXIBLE')
        out = [
            '  THE FRAMEWORK THAT WAS FLOWN',
            f'    agents / edges            {len(self.ids)} / {r["n_edges"]}',
            f'    rank R(p)                 {r["rank"]} of {r["needed"]} '
            f'needed -- {kind}',
            f'    rigidity margin           {r["margin"]:.3f}',
        ]
        if not r['rigid']:
            out += [
                '',
                '    WARNING: the target framework is not infinitesimally',
                f'    rigid -- {r["needed"] - r["rank"]} flex mode(s) beyond the '
                'trivial ones are',
                '    unpenalised by the distance constraints, so convergence to',
                '    this shape was never promised. Any shape error below is a',
                '    property of the graph, not a result about the hardware.',
            ]
        elif r['minimal']:
            out += [
                '    minimally rigid: |E| = 2n-3, so the graph is rigid but not',
                '    globally rigid and incorrect equilibria (a folded corner)',
                '    exist. Read the shape error, not the edge error.',
            ]
        return out

    def summarise(self, t, data, pos_hist):
        if not self.ok:
            return ['  the drone_testbed package was not importable, so the '
                    'shape analysis could not run']
        c = {name: i for i, name in enumerate(self.columns())}
        tail = slice(max(0, len(t) - max(1, int(0.1 * len(t))) - 1), None)

        edge_rms = data[:, c['edge_rms']]
        shape = data[:, c['shape_err']]
        d_min = data[:, c['d_min']]
        W = data[:, c['W_lyap']]
        worst = int(np.argmin(d_min))

        out = ['DISTANCE-BASED FORMATION -- did the theorem\'s promises hold?',
               '']
        out += self._rigidity_lines()

        out += [
            '',
            '  EDGE CONSTRAINTS (the quantity the law minimises)',
            f'    final rms error           {edge_rms[tail].mean() * 1000:.2f} mm',
            f'    final worst edge          {data[tail, c["edge_max"]].mean() * 1000:.2f} mm',
            f'    peak during run           {data[:, c["edge_max"]].max() * 1000:.1f} mm',
        ]
        bad = np.where(edge_rms > self.SETTLE_TOL)[0]
        if len(bad) == 0:
            out.append('    already within tolerance at the first sample')
        elif bad[-1] < len(t) - 1:
            out.append(f'    settled below {self.SETTLE_TOL * 1000:.0f} mm rms   '
                       f'at t = {t[bad[-1] + 1]:.1f} s')
        else:
            out.append('    NEVER settled -- still above tolerance at the end')

        final_shape = float(shape[tail].mean())
        mirrored = data[tail, c['mirrored']].mean() > 0.5
        out += [
            '',
            '  WAS IT THE RIGHT SHAPE? (rigid fit of the nominal onto the flown',
            '  positions; this is the metric edge error cannot give you)',
            f'    final residual            {final_shape * 1000:.2f} mm rms',
            f'    handedness                '
            f'{"MIRRORED vs the nominal" if mirrored else "same as the nominal"}',
        ]
        if mirrored:
            out.append('      -- not a fault. Ranges cannot distinguish a shape')
            out.append('         from its mirror image, whatever the graph.')
        if final_shape > self.SHAPE_TOL and edge_rms[tail].mean() < self.SETTLE_TOL:
            out += [
                '',
                '    *** INCORRECT EQUILIBRIUM ***',
                '    Every edge is satisfied and the shape is still wrong, so',
                '    the fleet came to rest at a configuration that is not the',
                '    one asked for. NOT a hardware failure -- check the topology',
                '    before attributing any of it to something physical.',
            ]
            out.append(
                '    The graph is flexible, so this was expected.'
                if not self.rigidity['rigid'] else
                '    The graph is rigid but not globally rigid, so this is the'
            )
            if self.rigidity['rigid']:
                out.append('    folded-corner equilibrium a minimal graph '
                           'admits. `octahedron`')
                out.append('    is globally rigid and does not have it.')
        elif final_shape > self.SHAPE_TOL:
            out.append('    shape not reached, and the edges have not settled '
                       'either -- still converging, or unstable')

        # -- Lyapunov -------------------------------------------------------
        #
        # The continuous law has W_dot = -kv sum||v||^2 <= 0. Anything that
        # makes W rise is the implementation, not the law: the zero-order hold
        # between control ticks, and the acceleration clamp truncating the
        # gradient. In the lab, differentiated-VICON velocity noise adds a
        # third source, so compare the rise here against the same config run
        # through tools/sim_baseline.py before blaming the hardware.
        dW = np.diff(W)
        # Once the fleet has settled, W sits at ~1e-17 and its differences are
        # pure float noise that alternates sign. Counting those as violations
        # reported "W rose on 0.9% of ticks" for a run that was in fact exactly
        # monotone, so a rise only counts if it is significant against the
        # scale of the run.
        floor = 1e-6 * max(float(W.max()), 1e-30)
        rises = dW[dW > floor]
        out += [
            '',
            '  LYAPUNOV FUNCTION W = potential + kinetic (must not increase)',
            f'    W at start / end          {W[0]:.4f} -> {W[-1]:.4f}',
            f'    ticks where W rose        {len(rises)} of {len(dW)} '
            f'({100.0 * len(rises) / max(len(dW), 1):.1f}%)   '
            f'[rises below {floor:.2e} ignored as float noise]',
        ]
        if len(rises):
            out.append(f'    largest single rise       {rises.max():.3e}'
                       f'  ({100.0 * rises.max() / max(float(W.max()), 1e-30):.2f}% of W_max)')
            out.append(f'    total energy injected     {rises.sum():.3e}')
            out.append('    W is not monotone. In simulation that is the '
                       'zero-order hold and the')
            out.append('    accel clamp (gap A); in the lab add velocity '
                       'estimation noise (gap B).')
            out.append('    Compare against tools/sim_baseline.py on the same '
                       'config to split them.')
        else:
            out.append('    monotonically decreasing, as the theory requires')

        # -- the invariants the law says are unobservable --------------------
        c0 = data[0, [c['cx'], c['cy']]]
        c1 = data[tail, :][:, [c['cx'], c['cy']]].mean(axis=0)
        drift = float(np.linalg.norm(c1 - c0))
        out += [
            '',
            '  CENTROID (interaction forces cancel in pairs, so from rest the',
            '  centroid cannot move; drift measures what broke that)',
            f'    start                     [{c0[0]:+.3f}, {c0[1]:+.3f}] m',
            f'    end                       [{c1[0]:+.3f}, {c1[1]:+.3f}] m',
            f'    drift                     {drift * 100:.2f} cm',
        ]
        if self.anchors:
            out.append('    NOTE: anchor is set, so the centroid was pinned '
                       'deliberately and this')
            out.append('          number is not a clean invariance test.')

        out += [
            '',
            '  ORIENTATION (free -- ranges cannot set an angle; recorded, not scored)',
            f'    final heading             {data[tail, c["orient_deg"]].mean():+.1f} '
            'deg from the nominal',
            '',
            '  SEPARATION',
            f'    smallest gap ever         {d_min[worst]:.4f} m  at t = {t[worst]:.1f} s',
            f'    ticks under {NEAR_MISS:.2f} m        '
            f'{int((d_min < NEAR_MISS).sum())} of {len(t)}',
            f'    final smallest gap        {d_min[tail].mean():.4f} m',
        ]
        return out


class GenericMetrics(MetricSet):
    """Fallback: positions and pairwise distances, for algorithms without a
    dedicated metric set. Enough to reconstruct most things after the fact."""

    def columns(self):
        cols = ['t']
        for i in self.ids:
            cols += [f'x_{i}', f'y_{i}', f'vx_{i}', f'vy_{i}']
        cols += [f'd_{self.ids[a]}_{self.ids[b]}' for a, b in self.pairs]
        cols += ['d_min']
        return cols

    def row(self, t, pos, vel):
        out = [t]
        for k in range(len(pos)):
            out += [float(pos[k, 0]), float(pos[k, 1]),
                    float(vel[k, 0]), float(vel[k, 1])]
        dists = [float(np.linalg.norm(pos[a] - pos[b])) for a, b in self.pairs]
        return out + dists + [min(dists)]

    def summarise(self, t, data, pos_hist):
        c = {name: i for i, name in enumerate(self.columns())}
        d_min = data[:, c['d_min']]
        return [
            'GENERIC METRICS (no dedicated set for this algorithm)',
            '',
            f'  smallest gap ever           {d_min.min():.4f} m  '
            f'at t = {t[int(np.argmin(d_min))]:.1f} s',
            f'  duration                    {t[-1] - t[0]:.1f} s',
        ]


def make_metrics(algo_name, ids, params):
    key = algo_name.lower()
    if key == 'flocking':
        return FlockingMetrics(ids, params)
    if key == 'coverage':
        return CoverageMetrics(ids, params)
    if 'trochoidal' in key:
        return TrochoidalMetrics(ids, params)
    if key == 'distanceformation':
        return DistanceFormationMetrics(ids, params)
    return GenericMetrics(ids, params)


# ---------------------------------------------------------------------------
# file format
# ---------------------------------------------------------------------------

def write_header(fh, cfg, algo, ids, cols, rate):
    """Provenance block, then a '#'-commented column ruler.

    Everything needed to interpret the run months later lives in the file
    itself -- which config, which params, which starting positions. A record
    that has to be matched back to a config by timestamp is a record nobody
    trusts.
    """
    w = fh.write
    w(f'# metrics_recorder -- {algo}\n')
    w(f'# started      {datetime.now().isoformat(timespec="seconds")}\n')
    w(f'# drones       {", ".join(ids)}\n')
    w(f'# sample rate  {rate} Hz\n')
    w('#\n# algorithm params:\n')
    for k, v in (cfg.get('algorithm', {}).get('params', {}) or {}).items():
        w(f'#   {k}: {v}\n')
    w('#\n# initial positions from config:\n')
    for d in cfg['drones']:
        w(f'#   {d["id"]}: {d.get("initial_position")}\n')
    w('#\n# columns:\n')
    for n, name in enumerate(cols, 1):
        w(f'#   {n:>2d}  {name}\n')
    w('#\n')
    # Shave two characters so the ruler lines up with the data rows despite
    # the '# ' comment prefix.
    w('# ' + ' '.join(f'{c:>16s}' for c in cols)[2:] + '\n')
    fh.flush()


def format_row(values):
    return ' '.join(f'{v:16.6f}' for v in values)


# ---------------------------------------------------------------------------
# node
# ---------------------------------------------------------------------------

class MetricsRecorder(Node):

    def __init__(self, cfg, path, rate, skip):
        super().__init__('metrics_recorder')
        self.ids = [d['id'] for d in cfg['drones']]
        algo_cfg = cfg.get('algorithm', {})
        self.algo = algo_cfg.get('name', 'Unknown')
        self.metrics = make_metrics(self.algo, self.ids,
                                    algo_cfg.get('params', {}) or {})
        self.path = path
        self.skip = skip

        self._pos = {i: None for i in self.ids}
        self._vel = {i: None for i in self.ids}
        self._t0 = None
        self._moving_at = None
        self._moving_since = None
        self._rows = []
        self._pos_hist = []
        self._aborted = None
        self._since_flush = 0

        for i in self.ids:
            self.create_subscription(
                Float64MultiArray, f'/{i}/state',
                lambda msg, did=i: self._state_cb(did, msg), 10)
        self.create_subscription(String, '/sim/abort', self._abort_cb, 10)

        self._fh = open(path, 'w')
        self._write_header(cfg, rate)

        self.create_timer(1.0 / rate, self._sample)
        self.get_logger().info(
            f'recording {self.algo} -> {path}  (Ctrl-C to stop and analyse)')

    # -- io ----------------------------------------------------------------

    def _write_header(self, cfg, rate):
        write_header(self._fh, cfg, self.algo, self.ids,
                     self.metrics.columns(), rate)

    def _state_cb(self, drone_id, msg):
        if len(msg.data) < 4:
            return
        self._pos[drone_id] = np.array(msg.data[0:2], dtype=float)
        self._vel[drone_id] = np.array(msg.data[2:4], dtype=float)

    def _abort_cb(self, msg):
        if self._aborted is None:
            self._aborted = msg.data
            self.get_logger().warn(f'abort seen: {msg.data}')

    def _sample(self):
        if any(self._pos[i] is None for i in self.ids):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        pos = np.array([self._pos[i] for i in self.ids])
        vel = np.array([self._vel[i] for i in self.ids])

        # Note when the fleet actually started moving, so the analysis can skip
        # the pre-flight hold that auto_start_delay leaves at the front. Motion
        # must persist for MOTION_HOLD before it counts -- a single sample over
        # the threshold is usually mocap noise on a drone still on the ground.
        if self._moving_at is None:
            if np.linalg.norm(vel, axis=1).max() > MOTION_SPEED:
                if self._moving_since is None:
                    self._moving_since = t
                elif t - self._moving_since >= MOTION_HOLD:
                    self._moving_at = self._moving_since
                    self.get_logger().info(
                        f'motion detected at t = {self._moving_at:.1f}s')
            else:
                self._moving_since = None

        row = self.metrics.row(t, pos, vel)
        if row is None:
            return
        self._rows.append(row)
        self._pos_hist.append(pos.copy())

        self._fh.write(format_row(row) + '\n')
        self._since_flush += 1
        if self._since_flush >= 10:
            self._fh.flush()
            self._since_flush = 0

    # -- analysis ----------------------------------------------------------

    def finish(self):
        if self._fh.closed:
            return
        self._fh.flush()
        if len(self._rows) < 4:
            self._fh.write('\n# too few samples to analyse\n')
            self._fh.close()
            print(f'\n[metrics] only {len(self._rows)} samples; '
                  f'raw data in {self.path}')
            return

        data = np.array(self._rows, dtype=float)
        t = data[:, 0]

        # Analyse from the moment of motion, not from node start.
        start = self.skip if self.skip is not None else (self._moving_at or 0.0)
        keep = t >= start
        if keep.sum() < 4:
            keep = np.ones(len(t), dtype=bool)
            start = t[0]

        lines = ['', '=' * 74, f'SUMMARY -- {self.algo}', '=' * 74, '']
        lines.append(f'  samples {int(keep.sum())} over '
                     f'{t[keep][-1] - t[keep][0]:.1f} s '
                     f'(analysis starts at t = {start:.1f} s)')
        if self._aborted:
            lines.append(f'  RUN ABORTED: {self._aborted}')
        lines.append('')
        try:
            lines += self.metrics.summarise(
                t[keep], data[keep],
                [self._pos_hist[i] for i in np.where(keep)[0]])
        except Exception as exc:            # noqa: BLE001
            lines.append(f'  analysis failed: {exc}')

        text = '\n'.join('# ' + ln if ln else '#' for ln in lines) + '\n'
        self._fh.write(text)
        self._fh.close()

        print('\n' + '\n'.join(lines))
        print(f'\n[metrics] written to {self.path}')


def reanalyse(path, cfg, t_from, t_to):
    """Re-score a finished record over a chosen time window.

    A real flight has three phases, and only the middle one is the experiment:
    a pre-flight hold while auto_start_delay runs out, the flight itself, and a
    tail after the real drones land while any virtual ones keep going. Scoring
    across all three reports the landing as a failure to settle and the virtual
    drones' post-landing wandering as inter-agent distances. Trim to the flight
    before drawing any conclusion.
    """
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    ids = [d['id'] for d in cfg['drones']]
    algo = cfg['algorithm']['name']
    metrics = make_metrics(algo, ids, cfg['algorithm'].get('params', {}) or {})

    expected = len(metrics.columns())
    if data.shape[1] != expected:
        # Records written before per-drone positions were added are still worth
        # reading -- everything except the wobble section works from the columns
        # they do have. Pad so the named lookups still line up, and say plainly
        # which analysis is unavailable rather than refusing the whole file.
        short = expected - data.shape[1]
        if short > 0 and short == 2 * len(ids):
            print(f'[metrics] {os.path.basename(path)} predates per-drone '
                  f'positions; wobble analysis unavailable for it.')
            data = np.hstack([data, np.full((len(data), short), np.nan)])
        else:
            raise SystemExit(
                f'{path} has {data.shape[1]} columns but {algo} expects '
                f'{expected} -- is --config the one this run used?')

    t = data[:, 0]
    lo = t_from if t_from is not None else t[0]
    hi = t_to if t_to is not None else t[-1]
    keep = (t >= lo) & (t <= hi)
    if keep.sum() < 4:
        raise SystemExit(f'only {int(keep.sum())} samples in [{lo}, {hi}]')

    # Coverage's optimality comparison needs starting positions, which the
    # recorded columns do not carry. Fall back to the config's marks.
    pos_hist = [np.array([d['initial_position'] for d in cfg['drones']],
                         dtype=float)]

    lines = ['', '=' * 74,
             f'RE-ANALYSIS -- {algo}', '=' * 74, '',
             f'  file    {path}',
             f'  window  t = {t[keep][0]:.1f} to {t[keep][-1]:.1f} s '
             f'({int(keep.sum())} of {len(t)} samples)', '']
    lines += metrics.summarise(t[keep], data[keep], pos_hist)
    print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(
        description='Record per-algorithm validation metrics.')
    ap.add_argument('--config', required=True,
                    help='the same testbed YAML the run uses')
    ap.add_argument('--analyse', metavar='FILE',
                    help='re-score an existing record instead of recording; '
                         'combine with --from / --to to trim to the flight')
    ap.add_argument('--from', dest='t_from', type=float, default=None,
                    help='--analyse only: first timestamp to include')
    ap.add_argument('--to', dest='t_to', type=float, default=None,
                    help='--analyse only: last timestamp to include')
    ap.add_argument('--out-dir', default='logs',
                    help='directory for the record file (default: logs/)')
    ap.add_argument('--rate', type=float, default=10.0,
                    help='samples per second (default: 10)')
    ap.add_argument('--skip', type=float, default=None,
                    help='seconds to exclude from the analysis; default is to '
                         'auto-detect when the fleet starts moving')
    a = ap.parse_args()

    with open(a.config) as f:
        cfg = yaml.safe_load(f)

    if a.analyse:
        reanalyse(a.analyse, cfg, a.t_from, a.t_to)
        return

    if not HAVE_ROS:
        raise SystemExit('recording needs ROS 2 -- source the workspace first. '
                         '(--analyse works without it.)')

    algo = cfg.get('algorithm', {}).get('name', 'unknown')
    os.makedirs(a.out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(a.out_dir, f'{algo.lower()}_{stamp}.txt')

    rclpy.init()
    node = MetricsRecorder(cfg, path, a.rate, a.skip)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
