#!/usr/bin/env python3
"""Record the metric that actually tests each algorithm's theorem.

A theorem promises a *property*, not a trajectory. Olfati-Saber never promises
a path -- it promises cohesion, no collisions, and matched velocities. So path
error tests something no paper claimed, and the quantity worth recording is
different for every algorithm. See docs/PROJECT_AIM.md section 7.

  Kuramoto    oscillator phases, wrapped pair differences, order parameter R,
              five-second synchronization hold, radius and position tracking.

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
    from std_msgs.msg import Float64MultiArray, Float64, Int32, String
    HAVE_ROS = True
except ImportError:                         # pragma: no cover
    HAVE_ROS = False
    Node = object

# Altitude comes straight from mocap rather than from /<id>/state, which only
# carries [x, y, vx, vy]. Separate guard: the interface package ships with
# Crazyswarm2 and is absent on a laptop, and a missing z is not a reason to
# refuse to record everything else.
try:
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.qos import qos_profile_sensor_data
    HAVE_MOCAP_MSG = True
except ImportError:                         # pragma: no cover
    HAVE_MOCAP_MSG = False


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

    # -- altitude ----------------------------------------------------------
    #
    # All three control laws are planar: z is pinned at the takeoff height by
    # crazyflie_node and never reaches the algorithm, which sees only
    # [x, y, vx, vy]. That makes altitude invisible in every metric here -- and
    # a drone quietly sinking out of its hover is exactly the kind of fault
    # that then gets blamed on the control law. Recorded for every algorithm,
    # appended last so no existing column changes index.

    def all_columns(self):
        return (self.columns() + [f'z_{i}' for i in self.ids]
                + [f'tilt_{i}' for i in self.ids])

    def full_row(self, t, pos, vel, z=None, tilt=None):
        r = self.row(t, pos, vel)
        if r is None:
            return None
        n = len(self.ids)

        def fill(values):
            if values is None:
                return [float('nan')] * n
            return [float('nan') if v is None else float(v) for v in values]
        return list(r) + fill(z) + fill(tilt)

    def altitude_lines(self, t, data):
        """Did every drone hold the height it took off at?"""
        base = len(self.columns())
        if data.shape[1] < base + len(self.ids):
            return []
        z = data[:, base:base + len(self.ids)]
        if not np.isfinite(z).any():
            return []                       # simulated run, or no mocap z

        out = ['', '  ALTITUDE (z from mocap; the algorithms never see it)',
               '    drone            held    drift    lowest   sink rate']
        for k, i in enumerate(self.ids):
            col = z[:, k]
            ok = np.isfinite(col)
            if ok.sum() < 4:
                out.append(f'    {i:12s}      --  (no mocap z)')
                continue
            tt, zz = t[ok], col[ok]
            # Fit a line: a steady sink shows up here even when the noise on
            # any single sample is larger than the drift itself.
            slope = float(np.polyfit(tt, zz, 1)[0])
            drift = float(zz[-1] - zz[0])
            flag = '  <-- SINKING' if slope < -0.005 else ''
            out.append(f'    {i:12s} {np.median(zz):6.3f}m '
                       f'{drift:+7.3f}m {zz.min():8.3f}m '
                       f'{slope * 1000:+7.1f} mm/s{flag}')
        out.append('')
        out.append('    "held" is the median height, "drift" is last minus first.')
        out.append('    A negative sink rate means the drone lost altitude over')
        out.append('    the run -- the commanded z is constant, so that is the')
        out.append('    onboard controller failing to hold it, not the algorithm.')
        return out

    def tilt_lines(self, t, data):
        """How far each real drone leaned, from the VICON orientation.

        Every control law here assumes a point mass that accelerates in any
        direction instantly. A quadrotor has to tilt by about atan(a/g) first,
        so tilt is the direct measure of how hard that assumption is loaded.
        """
        n = len(self.ids)
        base = len(self.columns()) + n
        if data.shape[1] < base + n:
            return []
        tilt = data[:, base:base + n]
        if not np.isfinite(tilt).any():
            return []
        out = ['', '  TILT (degrees from level, from the VICON orientation)',
               '    drone          median     p95     max']
        for k, i in enumerate(self.ids):
            col = tilt[:, k]
            col = col[np.isfinite(col)]
            if len(col) < 4:
                out.append(f'    {i:12s}      --  (no VICON body)')
                continue
            out.append(f'    {i:12s} {np.median(col):8.1f} '
                       f'{np.percentile(col, 95):7.1f} {col.max():7.1f}')
        out += ['', '    Point-mass reading: tilt ~ atan(|a|/g), so 1 m/s^2 of',
                '    acceleration is about 5.8 deg. The hover value is the',
                '    rigid body\'s own offset from level -- subtract it.']
        return out


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


class KuramotoMetrics(MetricSet):
    """Phase synchronization and physical tracking, with explicit data validity."""

    def __init__(self, ids, params):
        super().__init__(ids, params)
        self.center = np.asarray(params.get('center', [0., 0.]), dtype=float)
        self.radius = float(params.get('radius', .65))
        self.offsets = 2 * np.pi * np.arange(len(ids)) / len(ids)
        self.max_age = .5
        self.max_skew = .1
        self.sample_period = .1

    def columns(self):
        cols = ['t', 'active']
        for d in self.ids:
            cols += [f'phase_{d}', f'phase_age_{d}', f'state_age_{d}',
                     f'x_{d}', f'y_{d}', f'radius_{d}', f'radius_des_{d}',
                     f'position_err_{d}']
        cols += [f'phase_diff_{self.ids[a]}_{self.ids[b]}' for a, b in self.pairs]
        cols += [f'phase_offset_error_{self.ids[a]}_{self.ids[b]}' for a, b in self.pairs]
        return cols + ['phase_valid', 'order_R', 'phase_max_error',
                       'tracking_rms', 'radius_spread', 'd_min',
                       'angular_spacing_rms', 'angular_spacing_max_error', 'radius_rms']

    def row(self, t, pos, vel, phases=None, phase_ages=None, state_ages=None, active=False):
        n = len(self.ids)
        phases = np.full(n, np.nan) if phases is None else np.asarray(phases)
        phase_ages = np.full(n, np.inf) if phase_ages is None else np.asarray(phase_ages)
        state_ages = np.full(n, np.inf) if state_ages is None else np.asarray(state_ages)
        phase_fresh = np.isfinite(phases) & (phase_ages >= 0) & (phase_ages <= self.max_age)
        state_fresh = np.all(np.isfinite(pos), axis=1) & (state_ages >= 0) & (state_ages <= self.max_age)
        phase_valid = bool(phase_fresh.all() and np.ptp(phase_ages) <= self.max_skew)
        physical_valid = bool(state_fresh.all() and np.ptp(state_ages) <= self.max_skew)
        aligned = phase_valid and physical_valid and np.ptp(np.r_[phase_ages, state_ages]) <= self.max_skew
        radii = np.linalg.norm(pos - self.center, axis=1)
        desired = np.full(n, self.radius)
        targets = self.center + self.radius * np.column_stack([np.cos(phases), np.sin(phases)])
        errors = np.linalg.norm(pos - targets, axis=1)
        out = [t, float(active)]
        for k in range(n):
            local_valid = phase_fresh[k] and state_fresh[k] and abs(phase_ages[k] - state_ages[k]) <= self.max_skew
            out += [phases[k], phase_ages[k], state_ages[k], *pos[k],
                    radii[k] if state_fresh[k] else np.nan,
                    desired[k] if phase_fresh[k] else np.nan,
                    errors[k] if local_valid else np.nan]
        diffs = [float(np.arctan2(np.sin(phases[a] - phases[b]),
                                 np.cos(phases[a] - phases[b]))) if phase_valid else np.nan
                 for a, b in self.pairs]
        corrected = phases - self.offsets
        offset_errors = [float(np.arctan2(np.sin(corrected[a] - corrected[b]),
                                         np.cos(corrected[a] - corrected[b])))
                         if phase_valid else np.nan for a, b in self.pairs]
        order = float(abs(np.mean(np.exp(1j * corrected)))) if phase_valid else np.nan
        # Compare directed gaps in assigned drone order, including the last-to-first
        # edge. Angles at the center are undefined, so don't report spacing there.
        angles = np.arctan2(pos[:, 1] - self.center[1], pos[:, 0] - self.center[0])
        gap_error = np.roll(angles, -1) - angles - 2 * np.pi / n
        gap_error = np.arctan2(np.sin(gap_error), np.cos(gap_error))
        angular_valid = physical_valid and bool(np.all(radii > 1e-6))
        distances = [np.linalg.norm(pos[a] - pos[b]) for a, b in self.pairs]
        return out + diffs + offset_errors + [float(phase_valid), order,
                              max(map(abs, offset_errors), default=0.) if phase_valid else np.nan,
                              float(np.sqrt(np.mean(errors**2))) if aligned else np.nan,
                              float(np.std(radii)) if physical_valid else np.nan,
                              min(distances, default=np.nan) if physical_valid else np.nan,
                              float(np.sqrt(np.mean(gap_error**2))) if angular_valid else np.nan,
                              float(np.max(np.abs(gap_error))) if angular_valid else np.nan,
                              float(np.sqrt(np.mean((radii - self.radius)**2))) if physical_valid else np.nan]

    def summarise(self, t, data, pos_hist):
        c = {name: i for i, name in enumerate(self.columns())}
        active = data[:, c['active']] > 0
        order = data[:, c['order_R']]
        valid = active & np.isfinite(order)
        out = ['KURAMOTO -- oscillator synchronization and physical formation tracking',
               '  Phase/spacing units: radians; distance units: metres.',
               '  order_R and phase_max_error use phases minus assigned polygon offsets.',
               '  Latest receipt-time samples; no source timestamps or delay compensation.',
               f'  Validity limits: age <= {self.max_age}s, receipt skew <= {self.max_skew}s.',
               f'  valid active phase samples  {valid.sum()} of {active.sum()}']
        if valid.any():
            out += [f'  final valid order R         {order[valid][-1]:.6f}',
                    f'  mean active order R         {order[valid].mean():.6f}']
        # Missing/stale samples, stop/reset and recorder gaps break the hold.
        begin = None
        settled = None
        for k in range(len(t)):
            if not valid[k] or order[k] <= .99:
                begin = None
                continue
            if begin is None or (k and t[k] - t[k-1] > 1.5 * self.sample_period):
                begin = t[k]
            if t[k] - begin >= 5.:
                settled = begin
                break
        out.append('  first R > 0.99 held for 5s  ' +
                   (f't = {settled:.3f}s (record clock)' if settled is not None else 'not observed'))
        for name in ['phase_max_error', 'angular_spacing_rms', 'angular_spacing_max_error',
                     'tracking_rms', 'radius_rms', 'radius_spread', 'd_min']:
            values = data[active, c[name]]
            values = values[np.isfinite(values)]
            if len(values):
                out.append(f'  {name}: mean={values.mean():.6f}, min={values.min():.6f}, max={values.max():.6f}')
        for d in self.ids:
            values = data[active, c[f'radius_{d}']]
            values = values[np.isfinite(values)]
            if len(values):
                out.append(f'  {d} measured radius range  {values.min():.4f} .. {values.max():.4f} m')
        out.append('  Phase agreement alone does not establish physical rotating-ring tracking.')
        
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
    if key == 'kuramotoformation':
        return KuramotoMetrics(ids, params)
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

def write_header(fh, cfg, algo, ids, cols, rate, notes=None):
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
    # Launch settings that are not in the config (velocity window, artificial
    # noise, ...) and so would otherwise leave no trace in the record.
    for note in notes or []:
        w(f'# note         {note}\n')
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

    def __init__(self, cfg, path, rate, skip, notes=None, alias=None):
        super().__init__('metrics_recorder')
        self._notes = list(notes or [])
        self.ids = [d['id'] for d in cfg['drones']]
        algo_cfg = cfg.get('algorithm', {})
        self.algo = algo_cfg.get('name', 'Unknown')
        self.metrics = make_metrics(self.algo, self.ids,
                                    algo_cfg.get('params', {}) or {})
        self._phase = {i: float('nan') for i in self.ids}
        self._phase_time = {i: float('-inf') for i in self.ids}
        self._state_time = {i: float('-inf') for i in self.ids}
        self._active = False
        if isinstance(self.metrics, KuramotoMetrics):
            self.metrics.sample_period = 1.0 / rate
            for i in self.ids:
                self.create_subscription(Float64, f'/{i}/phase',
                                         lambda msg, did=i: self._phase_cb(did, msg), 10)
            self.create_subscription(Int32, '/sim/control', self._control_cb, 10)
        self.path = path
        self.skip = skip

        self._pos = {i: None for i in self.ids}
        self._vel = {i: None for i in self.ids}
        self._z = {i: None for i in self.ids}
        self._tilt = {i: None for i in self.ids}
        # Receipt times, for the TIMING block: how regularly VICON poses and
        # /state actually arrive, which the velocity fit silently depends on.
        self._pose_t = {i: [] for i in self.ids}
        self._pose_lat = {i: [] for i in self.ids}
        self._state_t = {i: [] for i in self.ids}
        # /poses names the VICON rigid bodies (drone_1), the testbed names the
        # logical agents (drone1). Nothing publishes the mapping, so accept
        # either spelling rather than making the operator pass a third list
        # that has to stay in step with cf_name and mocap_name.
        # Matching by number is WRONG whenever the two do not line up -- on
        # fig4, agent drone4 flies as VICON drone_2, and the 2026-09-09 records
        # filed its height under drone2. --hw/--mocap-name give the real map.
        self._z_alias = dict(alias or {})
        if not self._z_alias:
            for i in self.ids:
                for name in {i, i.replace('drone', 'drone_'), i.replace('_', '')}:
                    self._z_alias[name] = i
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
        if HAVE_MOCAP_MSG:
            self.create_subscription(NamedPoseArray, '/poses', self._poses_cb,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().warn(
                'motion_capture_tracking_interfaces not importable -- '
                'z will be recorded as nan')

        self._fh = open(path, 'w')
        self._write_header(cfg, rate)

        self.create_timer(1.0 / rate, self._sample)
        self.get_logger().info(
            f'recording {self.algo} -> {path}  (Ctrl-C to stop and analyse)')

    # -- io ----------------------------------------------------------------

    def _write_header(self, cfg, rate):
        write_header(self._fh, cfg, self.algo, self.ids,
                     (self.metrics.columns()
                      if isinstance(self.metrics, KuramotoMetrics)
                      else self.metrics.all_columns()),
                     rate, notes=self._notes)
        if isinstance(self.metrics, KuramotoMetrics):
            self._fh.write(
                '# Kuramoto: phases radians; positions/errors metres; ages seconds.\n'
                '# Latest receipt-time samples, not source-time synchronized.\n'
                '# Metrics require age <= 0.5s and receipt skew <= 0.1s.\n'
                '# Raw phases/positions retain last values; inspect ages for freshness.\n'
                '# active follows /sim/control; missing lifecycle start excludes summary.\n'
                '# Synchronization: R > 0.99 for 5s; invalid samples/gaps break hold.\n')
            self._fh.flush()

    def _state_cb(self, drone_id, msg):
        if len(msg.data) < 4 or not np.all(np.isfinite(msg.data[:4])):
            return
        now = self._now()
        self._state_time[drone_id] = now
        self._state_t[drone_id].append(now)
        self._pos[drone_id] = np.array(msg.data[0:2], dtype=float)
        self._vel[drone_id] = np.array(msg.data[2:4], dtype=float)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _poses_cb(self, msg):
        now = self._now()
        header = getattr(msg, 'header', None)
        stamp = (header.stamp.sec + header.stamp.nanosec * 1e-9
                 if header is not None else 0.0)
        for named in msg.poses:
            did = self._z_alias.get(named.name)
            if did is None:
                continue
            self._z[did] = float(named.pose.position.z)
            q = named.pose.orientation
            c = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)   # cos of the tilt angle
            self._tilt[did] = (math.degrees(math.acos(max(-1.0, min(1.0, c))))
                               if math.isfinite(c) else None)
            self._pose_t[did].append(now)
            if stamp > 0.0:
                self._pose_lat[did].append(now - stamp)

    def _phase_cb(self, drone_id, msg):
        if math.isfinite(msg.data):
            self._phase[drone_id] = msg.data
            self._phase_time[drone_id] = self.get_clock().now().nanoseconds * 1e-9

    def _control_cb(self, msg):
        self._active = msg.data == 1
        if msg.data == 2:
            self._phase_time = {i: float('-inf') for i in self.ids}
            self._state_time = {i: float('-inf') for i in self.ids}

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

        if isinstance(self.metrics, KuramotoMetrics):
            row = self.metrics.row(t, pos, vel,
                [self._phase[i] for i in self.ids],
                [now - self._phase_time[i] for i in self.ids],
                [now - self._state_time[i] for i in self.ids], self._active)
        else:
            # Every other algorithm keeps the z and tilt columns (appended
            # after its own, so no existing column index moves).
            row = self.metrics.full_row(t, pos, vel,
                                        [self._z[i] for i in self.ids],
                                        [self._tilt[i] for i in self.ids])
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

    def timing_lines(self):
        """How regularly VICON poses and /state really arrived.

        mocap_state_node fits velocity against the arrival times, so irregular
        arrival does not corrupt the slope -- but the fit is only as current as
        its window, and a window of N samples is (N-1)/2 samples old. This
        measures the rate that sets that lag, the jitter and the dropouts. The
        raw receipt times are saved next to the record as <record>_timing.npz.
        """
        out = ['', '  TIMING (receipt times on this machine)']
        found = False
        for i in self.ids:
            pt = np.array(self._pose_t[i])
            if len(pt) < 10:
                continue
            found = True
            dt = np.diff(pt) * 1000.0
            p50 = float(np.median(dt))
            gaps = int((dt > 20.0).sum())
            out.append(f'    {i:8s} VICON  {1000.0 / p50:6.1f} Hz   dt p50 {p50:5.1f}  '
                       f'p95 {np.percentile(dt, 95):5.1f}  max {dt.max():6.0f} ms   '
                       f'gaps >20 ms: {gaps} ({100.0 * gaps / len(dt):.1f}%)')
            st = np.array(self._state_t[i])
            if len(st) >= 10:
                sd = np.diff(st) * 1000.0
                out.append(f'    {"":8s} /state {1000.0 / np.median(sd):6.1f} Hz   '
                           f'dt p50 {np.median(sd):5.1f}  p95 {np.percentile(sd, 95):5.1f}  '
                           f'max {sd.max():6.0f} ms')
            lat = np.array(self._pose_lat[i])
            if len(lat) >= 10:
                out.append(f'    {"":8s} VICON stamp -> receipt  p50 '
                           f'{np.median(lat) * 1000:6.1f} ms  p95 '
                           f'{np.percentile(lat, 95) * 1000:6.1f} ms')
            out.append(f'    {"":8s} velocity-fit lag at this rate:  ' + '   '.join(
                f'window {n}: {(n - 1) / 2 * p50:4.0f} ms' for n in (5, 10, 20)))
        if not found:
            return []
        try:
            np.savez(self.path.replace('.txt', '_timing.npz'),
                     **{f'pose_{i}': np.array(self._pose_t[i]) for i in self.ids},
                     **{f'state_{i}': np.array(self._state_t[i]) for i in self.ids})
        except Exception:                   # noqa: BLE001
            pass
        return out

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
        if isinstance(self.metrics, KuramotoMetrics) and self.skip is None:
            start = 0.0  # Lifecycle column excludes pre-start rows from sync analysis.
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
        try:
            lines += self.metrics.altitude_lines(t[keep], data[keep])
        except Exception as exc:            # noqa: BLE001
            lines.append(f'  altitude analysis failed: {exc}')
        try:
            lines += self.metrics.tilt_lines(t[keep], data[keep])
        except Exception as exc:            # noqa: BLE001
            lines.append(f'  tilt analysis failed: {exc}')
        try:
            lines += self.timing_lines()
        except Exception as exc:            # noqa: BLE001
            lines.append(f'  timing analysis failed: {exc}')

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

    expected = len(metrics.all_columns())
    if data.shape[1] != expected:
        # Older records are still worth reading. The columns have only ever
        # grown at the end -- per-drone x/y, then z -- so a file short by whole
        # per-drone blocks is an earlier layout, not a mismatched config. Pad
        # with nan so the named lookups still line up, and name the analysis
        # that is unavailable instead of refusing the whole file.
        short = expected - data.shape[1]
        n = len(ids)
        # Blocks in the order they were added, newest first: tilt, z, and the
        # flocking per-drone x/y (two columns per drone).
        missing, left = [], short
        for name, width in (('tilt', n), ('altitude', n), ('wobble', 2 * n)):
            if left <= 0:
                break
            missing.append(name)
            left -= width
        if short > 0 and left == 0:
            print(f'[metrics] {os.path.basename(path)} predates the '
                  f'{" and ".join(missing)} column(s); that analysis is '
                  f'unavailable for it.')
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
    lines += metrics.altitude_lines(t[keep], data[keep])
    lines += metrics.tilt_lines(t[keep], data[keep])
    print('\n'.join(lines))


def installed_config(config_path):
    """The copy of this config that the flight will actually load, or None.

    algorithm_manager resolves config:=config/<name>.yaml against the package
    SHARE directory -- the copy colcon installed -- not against the file this
    recorder was handed. When the two differ, a header built from --config
    describes a flight that never happened. That is how the 2026-09-09
    "rescale 14" record came to carry rescale-14 params over a rescale-10
    flight (docs/PROJECT_AIM.md section 11b).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('drone_testbed')
    except Exception:
        return None
    path = os.path.join(share, 'config', os.path.basename(config_path))
    return path if os.path.exists(path) else None


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
    ap.add_argument('--note', action='append', default=[],
                    help='free text written into the header; repeatable. Use it '
                         'for launch settings the config cannot show, e.g. '
                         '--note "velocity_window 20" --note "mocap_noise 0.01"')
    ap.add_argument('--hw', default='',
                    help='the real drones, same list as the launch hw_drone:= '
                         '(e.g. drone1,drone4)')
    ap.add_argument('--mocap-name', default='',
                    help='their VICON names, same list as mocap_name:= (e.g. '
                         'drone_1,drone_2). Without --hw/--mocap-name, z and tilt '
                         'are matched by number, which mislabels fig4.')
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

    notes = list(a.note)
    hw = [x.strip() for x in a.hw.split(',') if x.strip()]
    mocap = [x.strip() for x in a.mocap_name.split(',') if x.strip()]
    if len(hw) != len(mocap):
        raise SystemExit(f'--hw {hw} and --mocap-name {mocap} must be the same '
                         f'length, matched positionally like the launch file')
    alias = dict(zip(mocap, hw))
    if alias:
        notes.append('real drones ' + ', '.join(f'{h}=VICON {m}' for m, h in alias.items()))
    twin = installed_config(a.config)
    if twin and os.path.realpath(twin) != os.path.realpath(a.config):
        with open(twin) as f:
            installed = yaml.safe_load(f)
        if (installed.get('algorithm') != cfg.get('algorithm')
                or installed.get('drones') != cfg.get('drones')):
            mine = (cfg.get('algorithm', {}) or {}).get('params', {}) or {}
            theirs = (installed.get('algorithm', {}) or {}).get('params', {}) or {}
            diffs = [f'{k}: {mine.get(k)} in --config, {theirs.get(k)} installed'
                     for k in sorted(set(mine) | set(theirs))
                     if mine.get(k) != theirs.get(k)]
            bar = '!' * 74
            print(f'\n{bar}\n'
                  f'  --config DIFFERS FROM THE INSTALLED COPY THE FLIGHT WILL LOAD\n'
                  f'    --config   {a.config}\n'
                  f'    installed  {twin}\n'
                  + ''.join(f'    {d}\n' for d in diffs) +
                  '  Recording the INSTALLED params, because those are what fly.\n'
                  '  If you meant to fly the new ones: stop, copy the yaml into\n'
                  '  ~/ros2_ws/src/drone_testbed/config/, colcon build, relaunch.\n'
                  f'{bar}\n')
            notes.append(f'--config {a.config} differed from the installed copy; '
                         f'the params below are the INSTALLED ones')
            cfg = installed

    algo = cfg.get('algorithm', {}).get('name', 'unknown')
    os.makedirs(a.out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(a.out_dir, f'{algo.lower()}_{stamp}.txt')

    if not HAVE_ROS:
        ap.error('ROS Python packages unavailable; source your ROS environment')
    if not math.isfinite(a.rate) or a.rate <= 0:
        ap.error('--rate must be positive and finite')
    rclpy.init()
    node = MetricsRecorder(cfg, path, a.rate, a.skip, notes, alias)
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
