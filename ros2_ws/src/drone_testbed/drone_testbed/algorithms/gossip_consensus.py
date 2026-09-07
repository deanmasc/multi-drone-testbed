"""Randomized gossip consensus -- formation control over ONE link per round.

Implements Boyd, Ghosh, Prabhakar & Shah, "Randomized Gossip Algorithms", IEEE
Transactions on Information Theory 52(6), 2006, section II, as the consensus
layer of a formation law.

    at each tick of a Poisson clock, ONE edge (i, j) wakes;
    both endpoints replace their value with the pairwise midpoint

        x(k+1) = W(k) x(k),    W(k) = I - (1/2)(e_i - e_j)(e_i - e_j)^T

Why this algorithm is in the testbed, given ConsensusFormation is already here.
The two solve the same task under opposite communication assumptions, and that
is the only difference between them:

  * ConsensusFormation assumes every agent reads every neighbour every control
    tick -- synchronous, all links live simultaneously, n^2 messages per second.
  * Gossip assumes ONE link carries ONE message per round, chosen at random,
    with no clock shared between agents and no routing. That is the regime the
    ReadMe's opening paragraph is about: the radio budget, not the control law,
    is what stops a fleet from scaling.

So the pair isolates the communication model the way Coverage and Trochoidal
isolate the stability type. Run both from the same starting positions and the
difference is attributable to the message schedule and nothing else.

WHAT THE THEOREM PROMISES (and therefore what to measure -- PROJECT_AIM.md s7)

  1. The average is preserved EXACTLY. Every W(k) is doubly stochastic, so
     1^T x(k) is invariant for every realisation of the randomness, not merely
     in expectation. The formation centre is thus fixed by the initial
     positions and can be written down before flying.

  2. Disagreement contracts at a rate set by the graph. W(k) is a projection,
     so E[W^T W] = E[W] = W_bar = I - (1/2) L_P, where L_P is the Laplacian
     weighted by the edge selection probabilities. Then

         E[|| x(k+1) - mean ||^2]  <=  lambda_2(W_bar) * || x(k) - mean ||^2

     i.e. SQUARED disagreement contracts by lambda_2 per round, so the RMS
     disagreement decays with per-round factor sqrt(lambda_2). Both numbers are
     printed by configure() below. For a complete graph on n agents with
     uniform edge selection lambda_2 = 1 - 1/(n - 1) exactly, which is 2/3 at
     n = 4.

     This is an expectation. One flight is one realisation, so a measured rate
     scatters around the prediction; judge it over several seeds, not one run.

HOW A DISCRETE MAP ON A VALUE BECOMES A CONTROL LAW -- read this before
believing any result from this file.

Gossip averages a *value*. A Crazyflie is a double integrator and cannot jump
to a midpoint. So each agent carries a gossip variable z_i in R^2 -- its
estimate of the formation anchor -- the paper's law runs faithfully and exactly
on z, and a PD wrapper flies the vehicle to its slot:

    a_i = kp * (z_i + d_i - p_i) - kd * v_i

The -kd*v term is OURS, exactly as in coverage.py, and for the same reason: it
makes a second-order agent behave like the first-order one the paper assumes.
It is an algorithm-level adaptation, not a harness one, so it deserves the
scrutiny PROJECT_AIM.md s5b reserves for that class. Two consequences:

  * z is software and jumps instantly; the drone lags behind it. The gap
    between the two is the whole sim-to-real story for this algorithm, and it
    is directly measurable -- the estimates satisfy the theorem exactly by
    construction, so any failure of the promised property is the vehicle's.
  * A gossip round steps the target of the two woken agents, which steps their
    commanded acceleration. Early rounds therefore saturate max_accel while the
    initial disagreement is large. It stops on its own: the step size decays
    geometrically with the disagreement.

`gossip_on: position` removes the estimate and gossips the physical position
instead -- the woken pair is commanded to the midpoint of where they actually
ARE. The average is then no longer preserved, because a drone never quite
arrives before the next round. Comparing the two modes is the single- vs
double-integrator question of PROJECT_AIM.md s9.1 asked of this algorithm.

HONEST LIMITATIONS, worth stating in the report:

  * The randomness is drawn from one RNG inside algorithm_manager. The paper's
    model is n INDEPENDENT local Poisson clocks; a real deployment would need a
    MAC layer to stop two pairs waking at once. Ours is a centralised stand-in
    for a distributed clock -- the arithmetic each agent does is local, the
    scheduling is not.
  * There is no collision term. Agents fly straight at their slots and the law
    will happily route two of them through the same point. Size the formation
    for separation and check it, as testbed_gossip.yaml does.
  * No setpoint is published. The target is a step function, so an explicit
    /cmd_pos would ask crazyflie_node to jump the drone to the new midpoint and
    trip the max_lead leash. The PD acceleration is the flyable path.

Config params:
  gossip_rate:    rounds per second. THE parameter of interest -- it is the
                  message budget. Deliberately decoupled from control_rate:
                  one round per control tick converges in about a second and
                  hides the asynchrony that is the point of the algorithm.
                  Default 0.5.
  poisson_clock:  true (default) draws the rounds due in each control tick from
                  Poisson(gossip_rate * dt), which is the paper's asynchronous
                  time model. false uses an evenly spaced schedule at the same
                  mean rate, so "does the exponential clock matter?" is a
                  config change.
  seed:           RNG seed, default 0. Fixed by default on purpose: an
                  unseeded randomised algorithm cannot be compared between
                  simulation and hardware. Set to null for a fresh draw.
  gossip_on:      'estimate' (default) | 'position' -- see above.
  gain_kp:        pull toward the agent's formation slot, default 1.0
  gain_kd:        velocity damping, default 1.2
  max_accel:      acceleration clamp (m/s^2), default 0.5, matches
                  crazyflie_node
  adjacency:      which pairs may gossip. Same two forms as the other
                  algorithms, but read as an UNDIRECTED graph, because a gossip
                  update writes to both endpoints:
                      drone1: ["drone2"]           unweighted, equal chance
                      drone1: {drone2: 5.0}        weighted
                  Weights are relative edge selection probabilities (the P_ij
                  of the paper); they are normalised to sum to 1. A weight
                  given in both directions is summed. Default: the complete
                  graph, every pair equally likely.
  target_offsets: dict of drone_id -> [dx, dy], the formation slot. Default is
                  a regular polygon of radius formation_radius.
  formation_radius: radius of that default polygon (m), default 0.6
  log_rounds:     print each gossip round as it happens, default False. Useful
                  in run_sim to watch which pair woke.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm
from drone_testbed.utils.types import DroneState, ControlOutput


# ── graph helpers ─────────────────────────────────────────────────────────────
#
# Module level, and taking plain ids rather than reading self, so that
# tools/metrics_recorder.py can rebuild the same graph from the same config and
# score a flight against the same prediction. Nothing here touches the vehicle.

def _weighted(neighbours) -> Dict[str, float]:
    """Normalise either adjacency form to {neighbour_id: weight}."""
    if isinstance(neighbours, dict):
        return {str(k): float(v) for k, v in neighbours.items()}
    return {str(k): 1.0 for k in neighbours}


def build_edges(
    drone_ids: Sequence[str],
    adjacency: Optional[dict] = None,
) -> Tuple[List[Tuple[int, int]], np.ndarray]:
    """Undirected edge list (as index pairs) and its selection probabilities.

    Gossip edges are undirected however they were written down: the update
    assigns the midpoint to BOTH endpoints, so a one-way entry still describes
    a two-way exchange. Listing a pair in both directions sums the weights,
    which only changes how often that pair is picked.
    """
    index = {d: i for i, d in enumerate(drone_ids)}
    n = len(drone_ids)
    weights: Dict[Tuple[int, int], float] = {}

    if adjacency:
        for drone_id, neighbours in adjacency.items():
            if drone_id not in index:
                continue
            for other, weight in _weighted(neighbours).items():
                if other not in index or other == drone_id:
                    continue
                weight = float(weight)
                if weight < 0.0:
                    raise ValueError(
                        f"adjacency weight {drone_id}->{other} is negative "
                        f"({weight}); weights are edge selection probabilities."
                    )
                a, b = index[drone_id], index[other]
                key = (min(a, b), max(a, b))
                weights[key] = weights.get(key, 0.0) + weight
    else:
        for a in range(n):
            for b in range(a + 1, n):
                weights[(a, b)] = 1.0

    edges = sorted(k for k, w in weights.items() if w > 0.0)
    if not edges:
        return [], np.zeros(0)
    probs = np.array([weights[e] for e in edges], dtype=float)
    return edges, probs / probs.sum()


def expected_gossip_matrix(
    n: int,
    edges: Sequence[Tuple[int, int]],
    probs: Sequence[float],
) -> np.ndarray:
    """W_bar = E[W] = I - (1/2) L_P, the paper's expected gossip matrix."""
    mat = np.eye(n)
    for (a, b), p in zip(edges, probs):
        vec = np.zeros(n)
        vec[a], vec[b] = 1.0, -1.0
        mat -= 0.5 * float(p) * np.outer(vec, vec)
    return mat


def contraction_factor(
    n: int,
    edges: Sequence[Tuple[int, int]],
    probs: Sequence[float],
) -> float:
    """lambda_2(W_bar): the per-round contraction of SQUARED disagreement.

    1.0 means no contraction at all (a disconnected or empty graph). The RMS
    disagreement decays by the square root of this per round.
    """
    if n < 2 or len(edges) == 0:
        return 1.0
    eig = np.sort(np.linalg.eigvalsh(
        expected_gossip_matrix(n, edges, probs)))[::-1]
    return float(eig[1])


def components(n: int, edges: Sequence[Tuple[int, int]]) -> int:
    """Number of connected components, by union-find."""
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return len({find(i) for i in range(n)})


def resolve_offsets(
    drone_ids: Sequence[str],
    params: dict,
) -> Dict[str, np.ndarray]:
    """Formation slot d_i for each agent, from config or a default polygon."""
    given = params.get('target_offsets', {}) or {}
    radius = float(params.get('formation_radius', 0.6))
    n = len(drone_ids)
    offsets = {}
    for i, drone_id in enumerate(drone_ids):
        if drone_id in given:
            offsets[drone_id] = np.array(given[drone_id], dtype=float)
        else:
            angle = 2.0 * math.pi * i / max(n, 1)
            offsets[drone_id] = np.array([
                radius * math.cos(angle),
                radius * math.sin(angle),
            ])
    return offsets


# ── the algorithm ─────────────────────────────────────────────────────────────

@register_algorithm
class GossipConsensus(BaseAlgorithm):

    def __init__(self):
        self._ids: List[str] = []
        self._offsets: Dict[str, np.ndarray] = {}
        self._edges: List[Tuple[int, int]] = []
        self._probs = np.zeros(0)

        self._kp = 1.0
        self._kd = 1.2
        self._max_accel = 0.5
        self._rate = 0.5
        self._poisson = True
        self._seed = 0
        self._mode = 'estimate'
        self._log_rounds = False

        self._rng = np.random.default_rng(0)
        self._z: Dict[str, np.ndarray] = {}
        self._pending = 0.0
        self._rounds = 0
        self._time = 0.0

    def name(self) -> str:
        return "GossipConsensus"

    # -- setup ---------------------------------------------------------------

    def configure(self, params: dict, drone_ids: List[str]) -> None:
        self._kp = float(params.get('gain_kp', 1.0))
        self._kd = float(params.get('gain_kd', 1.2))
        self._max_accel = float(params.get('max_accel', 0.5))
        self._rate = float(params.get('gossip_rate', 0.5))
        self._poisson = bool(params.get('poisson_clock', True))
        self._seed = params.get('seed', 0)
        self._log_rounds = bool(params.get('log_rounds', False))

        self._mode = str(params.get('gossip_on', 'estimate')).lower()
        if self._mode not in ('estimate', 'position'):
            raise ValueError(
                f"gossip_on must be 'estimate' or 'position', got "
                f"'{self._mode}'."
            )
        if self._rate <= 0.0:
            raise ValueError(
                f"gossip_rate ({self._rate}) must be positive; at zero no pair "
                "ever wakes and the drones hold their starting positions."
            )

        self._ids = list(drone_ids)
        self._offsets = resolve_offsets(self._ids, params)
        self._edges, self._probs = build_edges(
            self._ids, params.get('adjacency'))

        self.reset()
        self._report()

    def _report(self) -> None:
        """State the predictions before anything flies.

        Both are checkable against a recorded flight by
        tools/metrics_recorder.py, and both come from the graph alone -- no
        simulation and no gains involved.
        """
        n = len(self._ids)
        if n < 2:
            print('[GossipConsensus] fewer than 2 agents -- no edge can wake, '
                  'so nothing will move.')
            return
        if not self._edges:
            print('[GossipConsensus] the adjacency graph has no edges; no pair '
                  'can gossip and the drones will hold position.')
            return

        parts = components(n, self._edges)
        lam2 = contraction_factor(n, self._edges, self._probs)
        rms = math.sqrt(max(lam2, 0.0))

        print(f'[GossipConsensus] {len(self._edges)} gossip edges over {n} '
              f'agents, {self._rate:g} rounds/s '
              f'({"Poisson" if self._poisson else "fixed"} clock), '
              f'gossiping the {self._mode}')
        print(f'                  lambda_2(W_bar) = {lam2:.4f} '
              f'-> squared disagreement contracts by that per round,')
        print(f'                  RMS disagreement by {rms:.4f} per round')
        if rms <= 0.0:
            # Two agents on one edge: a single round puts both exactly on the
            # midpoint and there is nothing left to converge. lambda_2 = 0 is
            # correct, not a degenerate case, but it has no decay rate.
            print('                  predicted: exact consensus in a single '
                  'round (nothing left to decay)')
        elif rms < 1.0:
            per_round = math.log(rms)
            print(f'                  predicted: {-1.0 / per_round:.1f} rounds '
                  f'per e-fold, {math.log(0.05) / per_round:.0f} rounds to 5% '
                  f'({math.log(0.05) / per_round / self._rate:.0f} s at this '
                  f'rate)')
        if parts > 1:
            print(f'                  WARNING graph has {parts} components; '
                  f'each converges to its OWN average, so the formation will '
                  f'break into {parts} pieces.')

    def reset(self) -> None:
        # Re-seeding here rather than in configure is what makes a reset
        # reproduce the previous run instead of continuing the same stream.
        self._rng = np.random.default_rng(self._seed)
        self._z = {}
        self._pending = 0.0
        self._rounds = 0
        self._time = 0.0

    # -- the clock -----------------------------------------------------------

    def _rounds_due(self, dt: float) -> int:
        """How many gossip rounds fall inside this control tick.

        The paper's model is a Poisson clock per node; the superposition of
        those is itself a Poisson process, so drawing the count per tick from
        Poisson(rate * dt) is the model rather than an approximation of it. The
        fixed alternative accumulates fractional rounds so the mean rate is
        identical -- only the jitter differs.
        """
        expected = self._rate * dt
        if self._poisson:
            return int(self._rng.poisson(expected))
        self._pending += expected
        whole = int(self._pending)
        self._pending -= whole
        return whole

    # -- one gossip round ----------------------------------------------------

    def _gossip_once(self, states: Dict[str, DroneState]) -> None:
        """Wake one edge and average its two endpoints. The paper's line."""
        k = int(self._rng.choice(len(self._edges), p=self._probs))
        a, b = self._edges[k]
        id_a, id_b = self._ids[a], self._ids[b]

        # A drone that has not reported state yet cannot exchange anything. The
        # clock still ticked, so the round is spent -- which is the honest
        # model of a dropped message, not a bug to paper over.
        if id_a not in states or id_b not in states:
            return

        if self._mode == 'position':
            val_a = states[id_a].position - self._offsets[id_a]
            val_b = states[id_b].position - self._offsets[id_b]
        else:
            val_a = self._z[id_a]
            val_b = self._z[id_b]

        midpoint = 0.5 * (val_a + val_b)
        self._z[id_a] = midpoint.copy()
        self._z[id_b] = midpoint.copy()
        self._rounds += 1

        if self._log_rounds:
            print(f'[GossipConsensus] t={self._time:6.2f}s  round '
                  f'{self._rounds:4d}  {id_a} <-> {id_b}  -> '
                  f'({midpoint[0]:+.3f}, {midpoint[1]:+.3f})')

    # -- control -------------------------------------------------------------

    def compute_controls(
        self,
        states: Dict[str, DroneState],
        dt: float,
    ) -> Dict[str, ControlOutput]:
        self._time += dt

        # Seed each estimate from where its drone actually is, so the first
        # tick has zero position error and nothing lunges. This is also what
        # makes the preserved average -- and therefore the final formation
        # centre -- a function of the real starting positions.
        for drone_id, state in states.items():
            if drone_id not in self._z:
                self._z[drone_id] = (state.position
                                     - self._offsets.get(drone_id, np.zeros(2)))

        if self._edges:
            for _ in range(self._rounds_due(dt)):
                self._gossip_once(states)

        controls = {}
        for drone_id, state in states.items():
            target = (self._z[drone_id]
                      + self._offsets.get(drone_id, np.zeros(2)))

            # PD onto a piecewise-constant target: proportional to the slot
            # error, damped against the drone's own velocity. There is no
            # neighbour velocity term because between rounds there is no
            # neighbour -- that is the difference from ConsensusFormation.
            accel = self._kp * (target - state.position) - self._kd * state.velocity

            controls[drone_id] = ControlOutput(
                drone_id=drone_id,
                acceleration=np.clip(accel, -self._max_accel, self._max_accel),
                # No explicit setpoint on purpose. The target steps whenever
                # this agent gossips, so publishing it would ask crazyflie_node
                # to jump the drone to the new midpoint and trip the max_lead
                # leash. Integrating the acceleration is the flyable path.
            )

        return controls

    # -- introspection, for tools and tests -----------------------------------

    @property
    def rounds(self) -> int:
        """Gossip rounds executed since the last reset."""
        return self._rounds

    def estimates(self) -> Dict[str, np.ndarray]:
        """Current gossip variable per agent -- the quantity the theorem is
        about, as distinct from where the drones have managed to fly."""
        return {k: v.copy() for k, v in self._z.items()}

    def predicted_centre(self) -> Optional[np.ndarray]:
        """The formation centre every agent converges on.

        Exact for gossip_on='estimate': the mean of the offset-corrected
        starting positions is invariant under every gossip round, so this is
        fixed the moment the drones are placed.
        """
        if not self._z:
            return None
        return np.mean(np.array(list(self._z.values())), axis=0)
