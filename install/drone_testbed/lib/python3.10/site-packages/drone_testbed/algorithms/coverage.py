"""Voronoi coverage control (Lloyd's algorithm) for double-integrator agents.

Each drone drives toward the centroid of its own Voronoi cell, weighted by a
density function phi over a square region Q. The fleet converges to a
*centroidal Voronoi configuration* -- a local minimum of the locational cost

    H(p) = sum_i  integral_{V_i} ||q - p_i||^2 phi(q) dq

which is the standard measure of how well a set of sensors covers an area.
Lower H means every point of Q is closer to some agent.

Distributed: an agent's cell is bounded only by the agents whose cells touch it
(its Delaunay neighbours). This class is handed every position by
algorithm_manager, but the half-plane of a far-away agent never clips the cell,
so the result -- and the per-agent computation, which is what would run
on-board -- depends only on neighbours.

Reference:
  J. Cortes, S. Martinez, T. Karatas, F. Bullo, "Coverage control for mobile
  sensing networks", IEEE Transactions on Robotics and Automation, 20(2),
  243-255, 2004.

Config params:
  region_half_width: half-side of the square coverage region Q, metres.
                     Default 1.5. Keep this <= the geofence or agents will be
                     pulled toward centroids they are not allowed to reach.
  gain_kp:   pull toward the cell centroid, default 1.0
  gain_kd:   velocity damping, default 1.2
  max_accel: acceleration clamp (m/s^2), default 0.5 -- matches crazyflie_node
  grid_res:  samples per axis for the density integral over each cell.
             Default 36. Higher is smoother but O(n^2) slower per tick.
  density:   "uniform" | "gaussian", default "uniform"
  hotspot_sigma:  gaussian bump width (m), default 0.5
  hotspot_center: [x, y] the bump is centred on (or orbits), default [0.0, 0.0]
  hotspot_speed:  rad/s. Nonzero makes the bump orbit hotspot_center at
                  hotspot_orbit_radius -- a moving region of interest the swarm
                  has to track. Default 0.0 (stationary).
  hotspot_orbit_radius: m, default 0.6
  log_file:     path for a cost-history log; "" = timestamped file in the
                working directory, None/absent = no log. Default absent.
  log_interval: seconds between log samples, default 0.5
"""

import os
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


def _clip_halfplane(poly: np.ndarray, m: np.ndarray, d: np.ndarray
                    ) -> Optional[np.ndarray]:
    """Sutherland-Hodgman clip of a convex polygon by a half-plane.

    Keeps the part of `poly` satisfying (q - m) . d <= 0, i.e. the side of the
    perpendicular bisector between two agents that is closer to the first one.
    Input and output vertices are counter-clockwise; None if nothing survives.
    """
    out = []
    n = len(poly)
    for k in range(n):
        a = poly[k]
        b = poly[(k + 1) % n]
        da = float(np.dot(a - m, d))
        db = float(np.dot(b - m, d))
        if da <= 0.0:
            out.append(a)
        if (da < 0.0) != (db < 0.0):
            t = da / (da - db)
            out.append(a + t * (b - a))
    if len(out) < 3:
        return None
    return np.asarray(out)


def _points_in_convex(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Boolean mask of which points lie inside a CCW convex polygon."""
    inside = np.ones(len(pts), dtype=bool)
    n = len(poly)
    for k in range(n):
        a = poly[k]
        b = poly[(k + 1) % n]
        edge = b - a
        rel = pts - a
        cross = edge[0] * rel[:, 1] - edge[1] * rel[:, 0]
        inside &= cross >= -1e-12
    return inside


@register_algorithm
class Coverage(BaseAlgorithm):

    def __init__(self):
        self._half = 1.5
        self._kp = 1.0
        self._kd = 1.2
        self._max_accel = 0.5
        self._grid = 36
        self._density = 'uniform'
        self._sigma = 0.5
        self._hotspot_center = np.zeros(2)
        self._hotspot_speed = 0.0
        self._hotspot_orbit_radius = 0.6
        self._square = self._region_square(self._half)
        self._time = 0.0
        self._log = None
        self._log_interval = 0.5
        self._next_log = 0.0

    def name(self) -> str:
        return "Coverage"

    @staticmethod
    def _region_square(half: float) -> np.ndarray:
        return np.array([
            [-half, -half],
            [half, -half],
            [half, half],
            [-half, half],
        ], dtype=float)

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._half = float(params.get('region_half_width', 1.5))
        self._kp = float(params.get('gain_kp', 1.0))
        self._kd = float(params.get('gain_kd', 1.2))
        self._max_accel = float(params.get('max_accel', 0.5))
        self._grid = int(params.get('grid_res', 36))
        self._density = str(params.get('density', 'uniform')).lower()
        self._sigma = float(params.get('hotspot_sigma', 0.5))
        self._hotspot_center = np.asarray(
            params.get('hotspot_center', [0.0, 0.0]), dtype=float)
        self._hotspot_speed = float(params.get('hotspot_speed', 0.0))
        self._hotspot_orbit_radius = float(
            params.get('hotspot_orbit_radius', 0.6))
        self._square = self._region_square(self._half)
        self._log_interval = float(params.get('log_interval', 0.5))
        self.reset()
        if 'log_file' in params:
            self._open_log(params.get('log_file') or '')

    # ---- density -----------------------------------------------------------

    def _hotspot_at(self, t: float) -> np.ndarray:
        if self._hotspot_speed == 0.0:
            return self._hotspot_center
        ang = self._hotspot_speed * t
        return self._hotspot_center + self._hotspot_orbit_radius * np.array(
            [np.cos(ang), np.sin(ang)])

    def _phi(self, pts: np.ndarray, t: float) -> np.ndarray:
        """Importance weight at each query point. Strictly positive."""
        if self._density == 'gaussian':
            c = self._hotspot_at(t)
            r2 = ((pts - c) ** 2).sum(axis=1)
            # 1e-3 floor: far-from-hotspot cells still get a well-defined
            # centroid (~ the plain geometric one) instead of 0/0, so those
            # agents spread out and cover while the near ones pack in.
            return 1e-3 + np.exp(-r2 / (2.0 * self._sigma ** 2))
        return np.ones(len(pts))

    # ---- per-cell integral ----------------------------------------------

    def _cell_integral(self, poly: np.ndarray, p_i: np.ndarray, t: float
                       ) -> Optional[Tuple[np.ndarray, float]]:
        """Weighted centroid of a cell, and its contribution to H.

        Rectangular midpoint rule over the cell's bounding box, masked to the
        cell. The per-sample area cancels in the centroid but is kept for the
        cost so H comes out in real units.
        """
        lo = poly.min(axis=0)
        hi = poly.max(axis=0)
        gx = np.linspace(lo[0], hi[0], self._grid)
        gy = np.linspace(lo[1], hi[1], self._grid)
        mesh_x, mesh_y = np.meshgrid(gx, gy)
        pts = np.column_stack([mesh_x.ravel(), mesh_y.ravel()])

        inside = _points_in_convex(pts, poly)
        if not inside.any():
            return None
        pts = pts[inside]

        w = self._phi(pts, t)
        w_sum = float(w.sum())
        if w_sum <= 1e-12:
            return None

        centroid = (w[:, None] * pts).sum(axis=0) / w_sum

        da = ((hi[0] - lo[0]) / (self._grid - 1)) * \
             ((hi[1] - lo[1]) / (self._grid - 1))
        cost = float((w * ((pts - p_i) ** 2).sum(axis=1)).sum() * da)
        return centroid, cost

    # ---- main -----------------------------------------------------------

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        self._time += dt

        ids = list(states)
        pos = np.array([states[d].position for d in ids], dtype=float)

        # Break exact position ties (two agents on the same point make a
        # degenerate bisector). Deterministic so runs are reproducible.
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                if np.allclose(pos[a], pos[b], atol=1e-6):
                    pos[b] = pos[b] + 1e-4 * (b - a)

        controls: Dict[str, ControlOutput] = {}
        total_H = 0.0
        have_H = False

        for k, drone_id in enumerate(ids):
            vel = states[drone_id].velocity

            poly: Optional[np.ndarray] = self._square.copy()
            for j in range(len(ids)):
                if j == k:
                    continue
                d = pos[j] - pos[k]
                if float(np.dot(d, d)) < 1e-12:
                    continue
                m = 0.5 * (pos[k] + pos[j])
                poly = _clip_halfplane(poly, m, d)
                if poly is None:
                    break

            if poly is None:
                # No cell -- boxed in by coincident neighbours. Just damp.
                accel = -self._kd * vel
            else:
                result = self._cell_integral(poly, pos[k], self._time)
                if result is None:
                    accel = -self._kd * vel
                else:
                    centroid, cost_i = result
                    total_H += cost_i
                    have_H = True
                    accel = self._kp * (centroid - pos[k]) - self._kd * vel

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
            )

        if have_H:
            self._maybe_log(total_H, controls, states)
        return controls

    def reset(self) -> None:
        self._time = 0.0
        self._next_log = 0.0

    # ---- logging -------------------------------------------------------

    def _open_log(self, path: str) -> None:
        self._close_log()
        if not path:
            path = f'coverage_log_{datetime.now():%Y%m%d_%H%M%S}.txt'
        path = os.path.abspath(os.path.expanduser(path))
        try:
            self._log = open(path, 'w', buffering=1)
        except OSError as exc:
            print(f'[Coverage] could not open log file {path}: {exc}')
            return
        self._log.write(
            '# Coverage flight log. Locational cost H in m^2 (lower = better\n'
            '# coverage); time in seconds since the algorithm started.\n'
            '# t H mean_centroid_error\n'
        )
        print(f'[Coverage] logging cost history to {path}')

    def _close_log(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    def _maybe_log(self, H: float, controls, states) -> None:
        if self._log is None or self._time + 1e-9 < self._next_log:
            return
        self._next_log += self._log_interval
        if self._next_log < self._time:
            self._next_log = self._time + self._log_interval
        # Centroid error ~ |accel + kd v| / kp, i.e. how far each agent still is
        # from its cell centroid. Zero at a centroidal Voronoi configuration.
        errs = []
        for did, ctrl in controls.items():
            residual = ctrl.acceleration + self._kd * states[did].velocity
            errs.append(float(np.linalg.norm(residual)) / max(self._kp, 1e-6))
        self._log.write(f'{self._time:8.2f} {H:10.5f} {np.mean(errs):8.4f}\n')
