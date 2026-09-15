"""Distance-based (rigidity-based) formation control -- a hexagon.

Implements the gradient law of Krick, Broucke & Francis, "Stabilisation of
infinitesimally rigid formations of multi-robot networks", International
Journal of Control 82(3), 423-439, 2009, in the double-integrator form given by
Oh & Ahn, "Distance-based undirected formations of single-integrator and
double-integrator modeled agents in n-dimensional space", IJRNC 24(12),
1809-1820, 2014, eq. (20).

    e_ij = ||p_i - p_j||^2 - d_ij^2

    u_i  = -k_p * sum_{j in N_i} e_ij (p_i - p_j)  -  k_v * v_i

The first term is -grad_{p_i} V for the potential

    V(p) = (k_p / 4) * sum_{(i,j) in E} e_ij^2

so the fleet descends V, and V is the Lyapunov function of the proof. The
second term is the damping that carries the single-integrator result across to
double-integrator agents. Note that unlike `coverage.py` -- where the -k_d*v
wrapper is *ours* -- this damping is the published law, so there is no
algorithm-level adaptation to defend here (docs/PROJECT_AIM.md section 5b).

WHY THIS ALGORITHM IS IN THE TESTBED
------------------------------------
It occupies a corner of the sensing/information axis that nothing else here
does. Following the taxonomy of Oh, Park & Ahn, "A survey of multi-agent
formation control", Automatica 53 (2015) 424-440:

  ConsensusFormation  is *displacement-based*. Agent i is given the vector
                      d_j - d_i, so every agent must agree on which way is
                      north -- a common orientation of local frames.
  DistanceFormation   is *distance-based*. Agent i is given only the scalar
                      d_ij. No common frame, no common orientation. Each agent
                      could be running in its own arbitrarily rotated body
                      frame and the law is unchanged.

So this law asks strictly less of the sensing than the consensus law does, and
gets strictly less in return: a set of distances fixes the formation only up to
a rigid motion -- translation, rotation, and mirror image. Where the hexagon
ends up, which way it points, and which way round the drones run are all set by
the initial conditions, not by the controller, and no choice of graph or gain
changes that. Handedness in particular is worth being precise about, because it
is easy to expect global rigidity to fix it and it does not: global rigidity
says the realisation is unique *up to congruence*, and congruence includes
reflection. What it rules out is the different thing described under RIGIDITY
below -- a *subset* of the agents flipped relative to the rest, which is not a
congruence of the whole and does change the shape. So a globally rigid graph
promises the right hexagon, still at an arbitrary place, angle and handedness.
None of that is a bug to tune out; it is the price of dropping the common
frame, and measuring it is part of the point.

Three consequences worth predicting BEFORE flying (docs/PROJECT_AIM.md s10):

1. THE FORCE IS CUBIC IN THE ERROR, NOT LINEAR. Each edge contributes
   |r^2 - d^2| * r, which grows like r^3 for large r. Every other law in this
   directory is linear (consensus, leader-follower) or explicitly saturating
   (flocking's phi_alpha is bounded by construction). This one is not bounded
   at all, so the harness's `max_accel` clamp bites far harder and far sooner
   here than anywhere else. Prediction: distance-based formation control should
   be the algorithm most distorted by saturation, and the distortion should be
   confined to the transient -- once near the shape, r ~ d and the force is
   small again.

2. SATURATION BREAKS THE CENTROID INVARIANCE, AND THAT IS MEASURABLE IN
   SIMULATION. Because e_ij = e_ji, the interaction forces cancel exactly in
   pairs, so sum_i u_i^interaction = 0 and the fleet centroid obeys
   v_bar_dot = -k_v * v_bar. Starting from rest, the centroid CANNOT MOVE. The
   moment one agent's command is clipped and its partner's is not, the
   cancellation fails and the centroid walks. Centroid drift in simulation is
   therefore a pure gap-A measurement of how hard the clamp is working; drift
   on hardware is that plus gap B. `metrics_recorder.py` records it.

3. ONLY LOCAL CONVERGENCE IS PROMISED, AND INCORRECT EQUILIBRIA EXIST. The
   theorem is local: near the target shape, V is a valid Lyapunov function and
   the shape is asymptotically stable. Far from it, the fleet can descend into
   a different critical point of V -- a folded or collapsed configuration that
   satisfies every edge constraint but is not the shape you asked for. Whether
   that can happen is a property of the GRAPH, not of the gains (see below).
   This is why edge error alone is not a sufficient metric for this algorithm:
   an incorrect equilibrium reads as a perfect score on every edge. The shape
   has to be checked against the target directly, which is what the
   DistanceFormation metric set in tools/metrics_recorder.py does.

RIGIDITY -- WHY THE EDGE SET IS NOT A FREE CHOICE
-------------------------------------------------
The theorem requires the target framework to be *infinitesimally rigid*: the
rigidity matrix R(p) (one row per edge, holding (p_i - p_j) in agent i's block
and its negation in agent j's) must have rank 2n - 3 in the plane, the 3 being
the trivial motions -- two translations and one rotation. If the rank is short,
the shape has a continuous flex the distance constraints do not penalise, and
the fleet will drift along it forever without any edge error appearing.

A hexagonal ring alone is NOT rigid: six edges against the nine needed. Chords
have to be added, and which chords you add decides the behaviour. `configure`
computes the rank at the target shape and reports it, so a bad choice is caught
at startup instead of in the air. Measured for a regular hexagon:

  topology       |E|  rank  rigid?           notes
  ----------------------------------------------------------------------------
  cycle           6    6    NO               a hexagonal linkage; it flexes
  minimal         9    9    yes, minimally   ring + inner triangle; |E| = 2n-3
  k33             9    8    NO -- see below  generically rigid, degenerate here
  octahedron     12    9    yes, redundantly globally rigid; DEFAULT
  complete       15    9    yes              every distance known; not local

`k33` is the instructive one, and the reason the numerical check earns its
place. Taking the ring plus the three long diagonals gives K_{3,3}, which has
exactly 2n-3 = 9 edges and IS generically minimally rigid -- perturb the six
vertices and the rank comes back as 9. But K_{3,3} is infinitesimally flexible
precisely when its six vertices lie on a conic, and the vertices of a regular
hexagon lie on their circumcircle. So the one arrangement you actually want is
the measure-zero arrangement where the graph fails. A paper that says "assume
the framework is generic" is not wrong; it is just quietly excluding the
symmetric shape a reader would reach for first. This is exactly the
theory-to-practice gap the testbed exists to expose, so `k33` is kept as a
selectable topology rather than deleted.

`minimal` and `octahedron` are both infinitesimally rigid but differ in a way
that shows up in flight. `minimal` (ring + the inner triangle 1-3-5) is only
*locally* rigid: reflect drone2 across the line through drone1 and drone3 and
every one of the nine distances is still satisfied, so a hexagon with one
corner folded inward is a genuine equilibrium the fleet can reach from a bad
start -- and it reports zero edge error while it sits there. Simulated from the
clustered start in testbed_hexagon.yaml, that is exactly what happens: 224 mm
of shape error at a hundredth of a millimetre of edge error.

`octahedron` (ring plus all six two-hop chords, i.e. K_{2,2,2}) is redundantly
rigid and 4-connected, hence *globally* rigid: the folded corner now violates
one of the added chords, so it is no longer an equilibrium and the fleet
recovers the hexagon from the same start. That is why it is the default. It
costs each agent two extra range measurements and removes a whole class of
wrong answers -- though not, as noted above, the mirror image of the whole
formation, which no distance graph can exclude. Choosing `minimal` and watching
for folds is a legitimate experiment; choosing it by accident is not.

Config params:
  side_length:        side of the regular hexagon, metres. Also its
                      circumradius. Default 0.6.
  formation_center:   [x, y] centre of the nominal shape. Only used to place
                      the nominal vertices, which matter for `anchor` and for
                      the metrics' shape comparison -- the law itself never
                      sees an absolute position. Default [0, 0].
  formation_rotation: orientation of the nominal shape, radians. Same caveat:
                      the law cannot control orientation, so this only sets the
                      reference the metrics measure the final heading against.
  topology:           octahedron | minimal | k33 | cycle | complete | custom
                      Default octahedron.
  edges:              [[id_a, id_b], ...] -- required when topology is custom,
                      ignored otherwise.
  target_positions:   {drone_id: [x, y]} to define a shape that is not a
                      regular polygon. The desired distances are taken from
                      whatever these vertices imply.
  gain_kp:            gradient gain. Default 0.6.
  gain_kv:            damping. Default 1.0.
  max_accel:          per-axis clamp, m/s^2. Default 0.5, matching
                      crazyflie_node.
  anchor:             list of drone ids that ALSO receive absolute position
                      feedback toward their nominal vertex. Default [] -- the
                      faithful law, with translation and rotation both free.
                      This is a harness addition, not part of the published
                      law, so anything in it must be declared as such:
                        []           faithful; the shape parks wherever the
                                     initial centroid was, at whatever angle
                                     the initial conditions produced
                        ["drone1"]   pins translation only. One agent needs
                                     global position; the other five stay
                                     purely range-based. Rotation still free.
                        two or more  pins rotation and reflection as well, at
                                     which point it is no longer really a
                                     distance-based formation -- use it only to
                                     keep a lab flight inside the geofence.
  anchor_gain_kp:     anchor position gain. Default 0.4.
  anchor_gain_kd:     anchor damping, applied on top of gain_kv. Default 0.0.
"""

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


# Named chord sets, as offsets around the ring. `cycle` is always present.
TOPOLOGIES = ('octahedron', 'minimal', 'k33', 'cycle', 'complete', 'custom')


def regular_polygon(n: int, radius: float, center: Sequence[float],
                    rotation: float) -> np.ndarray:
    """Vertices of a regular n-gon, counter-clockwise from `rotation`.

    For a hexagon the circumradius equals the side length, so `radius` is also
    the inter-agent spacing along the ring.
    """
    c = np.asarray(center, dtype=float)
    return np.array([
        c + radius * np.array([math.cos(rotation + 2.0 * math.pi * k / n),
                               math.sin(rotation + 2.0 * math.pi * k / n)])
        for k in range(n)
    ])


def build_edges(topology: str, n: int) -> List[Tuple[int, int]]:
    """Edge set as index pairs, for a ring of n agents in config order.

    Raises on a topology that cannot be built for this n, rather than silently
    returning something differently shaped.
    """
    topology = str(topology).lower()
    cycle = [(k, (k + 1) % n) for k in range(n)]

    if topology == 'cycle':
        return cycle
    if topology == 'complete':
        return [(a, b) for a in range(n) for b in range(a + 1, n)]
    if topology == 'octahedron':
        # Ring plus every two-hop chord. n >= 5, or the chords duplicate ring
        # edges (n = 4) or collapse entirely (n = 3).
        if n < 5:
            raise ValueError(
                f"topology 'octahedron' needs at least 5 agents, got {n}; "
                "use 'complete' for a triangle or a square."
            )
        return cycle + [(k, (k + 2) % n) for k in range(n)]
    if topology == 'k33':
        if n % 2 or n < 4:
            raise ValueError(
                f"topology 'k33' needs an even number of agents >= 4, got {n}."
            )
        return cycle + [(k, k + n // 2) for k in range(n // 2)]
    if topology == 'minimal':
        # 2n - 3 edges, the fewest that can hold a shape in the plane.
        #
        # For the hexagon this is the ring plus the inner triangle 1-3-5: a
        # rigid triangle with three ears, each ear attached by two edges, which
        # is a Henneberg type-I construction and so minimally rigid. It is used
        # for n = 6 specifically because it is the symmetric answer -- maximum
        # degree 4 rather than the fan's n-1, so no agent is a hub -- and
        # because its flip ambiguity is the easy one to see: any ear can fold
        # inward across the triangle edge it straddles without changing a
        # single distance.
        #
        # For any other n that construction does not close, so fall back to a
        # fan of chords from agent 0, which is Henneberg-constructible and
        # gives exactly 2n - 3 edges for every n >= 3.
        if n == 6:
            return cycle + [(0, 2), (2, 4), (4, 0)]
        return cycle + [(0, k) for k in range(2, n - 1)]

    raise ValueError(
        f"unknown topology '{topology}'. Expected one of {', '.join(TOPOLOGIES)}."
    )


def rigidity_matrix(pos: np.ndarray,
                    edges: Sequence[Tuple[int, int]]) -> np.ndarray:
    """The |E| x 2n rigidity matrix R(p) of a planar framework."""
    n = len(pos)
    R = np.zeros((len(edges), 2 * n))
    for row, (i, j) in enumerate(edges):
        diff = pos[i] - pos[j]
        R[row, 2 * i:2 * i + 2] = diff
        R[row, 2 * j:2 * j + 2] = -diff
    return R


def rigidity_report(pos: np.ndarray,
                    edges: Sequence[Tuple[int, int]]) -> Dict[str, float]:
    """Rank of R(p) against the 2n - 3 needed, and how close the call was.

    `margin` is the singular value that has to stay away from zero for the
    framework to be rigid -- the smallest non-trivial one. A framework can pass
    the rank test on a tolerance and still be nearly flexible, which shows up
    as a soft mode the fleet drifts along; the margin is what tells them apart.
    Scale-dependent, so compare it between topologies at the same side length,
    not against an absolute threshold.
    """
    n = len(pos)
    needed = 2 * n - 3
    sv = np.linalg.svd(rigidity_matrix(pos, edges), compute_uv=False)
    tol = max(sv[0], 1.0) * 1e-9
    rank = int((sv > tol).sum())
    margin = float(sv[needed - 1]) if len(sv) >= needed else 0.0
    return {
        'rank': rank,
        'needed': needed,
        'rigid': rank == needed,
        'minimal': len(edges) == needed,
        'margin': margin,
        'n_edges': len(edges),
    }


def formation_spec(params: dict, drone_ids: Sequence[str]):
    """Target vertices, edge list and desired distances from a params block.

    Shared with tools/metrics_recorder.py so the analysis scores the flight
    against exactly the shape the controller was flying, rather than against a
    second copy of the geometry that can drift out of step with this one.

    Returns (targets, edges, dist) where targets is n x 2 in config order,
    edges is a list of index pairs, and dist[k] is the desired length of
    edges[k].
    """
    ids = list(drone_ids)
    n = len(ids)
    side = float(params.get('side_length', 0.6))
    center = params.get('formation_center', [0.0, 0.0])
    rotation = float(params.get('formation_rotation', 0.0))

    targets = regular_polygon(n, side, center, rotation)
    explicit = params.get('target_positions') or {}
    for k, drone_id in enumerate(ids):
        if drone_id in explicit:
            targets[k] = np.asarray(explicit[drone_id], dtype=float)

    topology = str(params.get('topology', 'octahedron')).lower()
    if topology == 'custom':
        pairs = params.get('edges') or []
        if not pairs:
            raise ValueError(
                "topology 'custom' requires a non-empty `edges` list of "
                "[drone_a, drone_b] pairs."
            )
        index = {d: k for k, d in enumerate(ids)}
        edges = []
        for pair in pairs:
            a, b = str(pair[0]), str(pair[1])
            if a not in index or b not in index:
                raise ValueError(
                    f"edge [{a}, {b}] names a drone that is not in the config."
                )
            if a == b:
                raise ValueError(f"edge [{a}, {b}] joins a drone to itself.")
            edges.append(tuple(sorted((index[a], index[b]))))
        edges = sorted(set(edges))
    else:
        edges = build_edges(topology, n)

    dist = np.array([np.linalg.norm(targets[i] - targets[j])
                     for i, j in edges])
    if (dist < 1e-6).any():
        raise ValueError(
            'two target vertices coincide, so an edge has zero desired '
            'length -- check side_length and target_positions.'
        )
    return targets, edges, dist


@register_algorithm
class DistanceFormation(BaseAlgorithm):

    def __init__(self):
        self._ids: List[str] = []
        self._targets = np.zeros((0, 2))
        self._edges: List[Tuple[int, int]] = []
        self._dist = np.zeros(0)
        # Per-agent adjacency as (neighbour_id, d^2). Squared once here because
        # the law only ever wants d^2, and squaring per edge per tick on the
        # lab machine is wasted budget (docs/PROJECT_AIM.md s9, experiment 6).
        self._neighbours: Dict[str, List[Tuple[str, float]]] = {}
        self._kp = 0.6
        self._kv = 1.0
        self._max_accel = 0.5
        self._anchor: Dict[str, np.ndarray] = {}
        self._anchor_kp = 0.4
        self._anchor_kd = 0.0
        self._rigidity: Dict[str, float] = {}

    def name(self) -> str:
        return "DistanceFormation"

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._ids = list(drone_ids)
        self._targets, self._edges, self._dist = formation_spec(
            params, self._ids)

        self._kp = float(params.get('gain_kp', 0.6))
        self._kv = float(params.get('gain_kv', 1.0))
        self._max_accel = float(params.get('max_accel', 0.5))
        self._anchor_kp = float(params.get('anchor_gain_kp', 0.4))
        self._anchor_kd = float(params.get('anchor_gain_kd', 0.0))

        self._neighbours = {d: [] for d in self._ids}
        for (i, j), d in zip(self._edges, self._dist):
            self._neighbours[self._ids[i]].append((self._ids[j], d * d))
            self._neighbours[self._ids[j]].append((self._ids[i], d * d))

        anchor_ids = [str(a) for a in (params.get('anchor') or [])]
        unknown = [a for a in anchor_ids if a not in self._ids]
        if unknown:
            raise ValueError(
                f"anchor names {unknown}, which are not drones in this config."
            )
        self._anchor = {
            a: self._targets[self._ids.index(a)].copy() for a in anchor_ids
        }

        self._rigidity = rigidity_report(self._targets, self._edges)
        self._report_rigidity(params)

    def _report_rigidity(self, params: dict) -> None:
        """Say at startup whether the requested shape is actually stabilisable.

        Printed rather than raised. A flexible framework is a legitimate thing
        to fly deliberately -- watching a hexagonal ring shear is the clearest
        demonstration of why rigidity is required at all -- but it must never
        happen without the operator having been told.
        """
        r = self._rigidity
        topology = str(params.get('topology', 'octahedron')).lower()
        kind = ('minimally rigid' if r['rigid'] and r['minimal']
                else 'rigid' if r['rigid'] else 'FLEXIBLE')
        print(
            f'[DistanceFormation] {len(self._ids)} agents, topology '
            f'{topology}, {r["n_edges"]} edges: rank(R) = {r["rank"]} '
            f'of {r["needed"]} needed -- {kind} '
            f'(margin {r["margin"]:.3f})'
        )
        if not r['rigid']:
            print(
                f'[DistanceFormation] WARNING: the target framework has '
                f'{r["needed"] - r["rank"]} flex mode(s) beyond the trivial '
                'translations and rotation. The distance constraints do not '
                'penalise motion along them, so the shape will deform with '
                'every edge error still reading zero. Convergence to the '
                'requested hexagon is NOT promised.'
            )
            if topology == 'k33':
                print(
                    '[DistanceFormation] k33 is generically minimally rigid; '
                    'it degenerates here because the six vertices of a regular '
                    'hexagon lie on a common conic (their circumcircle). '
                    'Perturb the target vertices and the rank recovers.'
                )

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        controls = {}

        for drone_id, state in states.items():
            neighbours = self._neighbours.get(drone_id)
            if neighbours is None:
                continue

            # -grad_{p_i} V, one term per incident edge. Nothing here reads an
            # absolute position or a common heading: only the vector to a
            # neighbour and a scalar target distance, which is the whole point
            # of the distance-based formulation.
            force = np.zeros(2)
            for neighbour_id, d_sq in neighbours:
                other = states.get(neighbour_id)
                if other is None:
                    continue
                z = state.position - other.position
                force -= (float(z @ z) - d_sq) * z

            accel = self._kp * force - self._kv * state.velocity

            # Optional absolute-position feedback -- ours, not the paper's.
            target = self._anchor.get(drone_id)
            if target is not None:
                accel += (-self._anchor_kp * (state.position - target)
                          - self._anchor_kd * state.velocity)

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
                # No setpoint: the shape is emergent and its placement is not
                # known to the controller, so there is no reference position to
                # publish. crazyflie_node integrates the command.
            )

        return controls

    def reset(self) -> None:
        pass
