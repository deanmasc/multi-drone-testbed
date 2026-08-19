#!/usr/bin/env python3
"""Compute gains for TrochoidalConsensus, and prove they do what they claim.

The gains in the paper's control law are not free parameters. A trochoid
appears only when the closed-loop matrix has exactly TWO distinct pairs of
eigenvalues on the imaginary axis and every other pole strictly in the left
half plane. Pick alpha/kappa by hand and you almost certainly get one of the
degenerate cases instead: decay to a point, a plain circle, or a spiral that
diverges. So they have to be solved for.

Given the interaction graph, a coupling angle theta and a damping beta, this
derives kappa and alpha from Monsingh, Sinha & Chung (2024) eq. (33) and (34),
then verifies the result numerically against the actual system matrix rather
than trusting the algebra -- the two frequencies it predicts must match the
imaginary parts of the eigenvalues, and nothing may sit in the right half plane.

It also simulates forward from the initial positions in the config to report
the steady-state envelope, because the pattern's SIZE comes from the initial
conditions, not from any gain, and that is what has to fit the geofence.

Usage:
    python3 tools/design_consensus_gains.py --drones 4
    python3 tools/design_consensus_gains.py --drones 4 --sweep
    python3 tools/design_consensus_gains.py --config \
        ros2_ws/src/drone_testbed/config/testbed_hybrid.yaml
"""

import argparse
import math

import numpy as np


def rotation(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def laplacian_cycle(n):
    """Directed cycle: agent i observes agent i+1."""
    adj = np.zeros((n, n))
    for i in range(n):
        adj[i, (i + 1) % n] = 1.0
    return np.diag(adj.sum(1)) - adj


def laplacian_from_adjacency(ids, adjacency):
    n = len(ids)
    index = {d: i for i, d in enumerate(ids)}
    adj = np.zeros((n, n))
    for d, neighbours in adjacency.items():
        for other in neighbours:
            if d in index and other in index:
                adj[index[d], index[other]] = 1.0
    return np.diag(adj.sum(1)) - adj


def system_matrix(lap, alpha, beta, kappa, theta):
    n = lap.shape[0]
    eye = np.eye(2 * n)
    return np.block([
        [np.zeros((2 * n, 2 * n)), eye],
        [-alpha * eye - kappa * np.kron(lap, rotation(theta)), -beta * eye],
    ])


def solve_gains(lam_k, lam_l, theta, beta):
    """Paper eq. (33) and (34): place two eigenvalue pairs on the imaginary axis.

    phi = arg(lam) - theta. Setting the two modes to share one alpha gives a
    linear equation in kappa, which is why the paper solves for this pair
    rather than for theta or beta.
    """
    phi_k, phi_l = np.angle(lam_k) - theta, np.angle(lam_l) - theta
    mag_k, mag_l = abs(lam_k), abs(lam_l)
    numerator = mag_k * np.cos(phi_k) - mag_l * np.cos(phi_l)
    denominator = (mag_k ** 2 * np.sin(phi_k) ** 2
                   - mag_l ** 2 * np.sin(phi_l) ** 2)
    if abs(denominator) < 1e-12:
        return None
    kappa = -beta ** 2 * numerator / denominator
    alpha = (kappa * mag_k * np.cos(phi_k)
             + kappa ** 2 * mag_k ** 2 * np.sin(phi_k) ** 2 / beta ** 2)
    omega_1 = abs(kappa * mag_k * np.sin(phi_k) / beta)
    omega_2 = abs(kappa * mag_l * np.sin(phi_l) / beta)
    return kappa, alpha, omega_1, omega_2


def verify(lap, alpha, beta, kappa, theta, tol=1e-7):
    """Check the claim against the real system matrix, not the algebra."""
    eig = np.linalg.eigvals(system_matrix(lap, alpha, beta, kappa, theta))
    on_axis = sorted({round(abs(z.imag), 4)
                      for z in eig if abs(z.real) < tol and abs(z.imag) > tol})
    unstable = int((eig.real > tol).sum())
    return on_axis, unstable, eig


def place_agents(lap, alpha, beta, kappa, theta, target_radius, tol=1e-7):
    """Solve for starting positions that actually produce a pattern of a given size.

    The gains decide the SHAPE of the trochoid; the initial conditions decide
    its size -- but not proportionally, and not intuitively. Most arrangements
    project almost entirely onto the modes that decay, so the drones spiral in
    and the surviving pattern is tiny no matter how far apart they started. A
    symmetric ring is one of the bad cases.

    The undamped modes are the eigenvectors of A for the imaginary-axis
    eigenvalues. Their position components span the arrangements that survive,
    so build the start from those and every bit of it feeds the pattern. The
    system is linear, so one simulation fixes the scale exactly.
    """
    n = lap.shape[0]
    mat = system_matrix(lap, alpha, beta, kappa, theta)
    values, vectors = np.linalg.eig(mat)

    # One representative per conjugate pair, positive imaginary part.
    modes = [vectors[:, i] for i, z in enumerate(values)
             if abs(z.real) < tol and z.imag > tol]
    if len(modes) < 2:
        return None

    # Position half of each mode, taken real. Summing the two undamped modes is
    # what superimposes two circular motions -- i.e. makes a trochoid rather
    # than a circle.
    start = np.zeros((n, 2))
    for mode in modes[:2]:
        start += np.real(mode[:2 * n]).reshape(n, 2)

    scale = np.abs(start).max()
    if scale < 1e-9:
        return None
    start = start / scale * target_radius

    # Linear system: measure what that start actually yields, then rescale once.
    tail = simulate(lap, alpha, beta, kappa, theta, start)[-20000:]
    got = max(np.hypot(tail[:, 2 * i], tail[:, 2 * i + 1]).max() for i in range(n))
    if got > 1e-9:
        start = start * (target_radius / got)
    return start


def simulate(lap, alpha, beta, kappa, theta, start, seconds=400.0, dt=0.005):
    n = lap.shape[0]
    coupling = -alpha * np.eye(2 * n) - kappa * np.kron(lap, rotation(theta))
    pos = np.array(start, dtype=float).reshape(-1)
    vel = np.zeros(2 * n)
    steps = int(seconds / dt)
    keep = np.zeros((steps, 2 * n))
    for k in range(steps):
        acc = coupling @ pos - beta * vel
        vel += acc * dt
        pos += vel * dt
        keep[k] = pos
    return keep


def report(lap, ids, alpha, beta, kappa, theta, start, geofence):
    on_axis, unstable, _ = verify(lap, alpha, beta, kappa, theta)
    print(f"\n  alpha={alpha:.4f}  beta={beta:.4f}  kappa={kappa:.4f}  "
          f"theta={theta:.4f}")
    print(f"  eigenvalues on the imaginary axis : {on_axis}")
    print(f"  poles in the right half plane     : {unstable}")
    if len(on_axis) != 2 or unstable:
        print("  -> NOT a trochoid. Need exactly 2 distinct frequencies "
              "and 0 unstable poles.")
        return False

    full = simulate(lap, alpha, beta, kappa, theta, start)
    tail = full[-20000:]
    transient = max(np.hypot(full[:, 2 * i], full[:, 2 * i + 1]).max()
                    for i in range(len(ids)))
    print(f"  periods                           : "
          f"{2 * math.pi / on_axis[0]:.1f}s and {2 * math.pi / on_axis[1]:.1f}s")
    print(f"  {'agent':<10} {'centre':>18} {'r_min':>7} {'r_max':>7} {'max|pos|':>9}")
    worst = 0.0
    for i, drone_id in enumerate(ids):
        x, y = tail[:, 2 * i], tail[:, 2 * i + 1]
        cx, cy = x.mean(), y.mean()
        radius = np.hypot(x - cx, y - cy)
        far = np.hypot(x, y).max()
        worst = max(worst, far)
        print(f"  {drone_id:<10} ({cx:+.3f},{cy:+.3f}) "
              f"{radius.min():7.3f} {radius.max():7.3f} {far:9.3f}")
    print(f"  steady-state envelope             : {worst:.3f} m")
    print(f"  peak INCLUDING transient          : {transient:.3f} m "
          f"({'inside' if transient < geofence else 'OUTSIDE'} the "
          f"{geofence} m geofence)")
    ok = transient < geofence
    if not ok:
        print("  -> scale the initial positions down; radius comes from them, "
              "not from the gains.")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drones', type=int, default=4)
    ap.add_argument('--beta', type=float, default=1.0)
    ap.add_argument('--theta', type=float, default=None,
                    help='coupling angle (rad); omit to search for one')
    ap.add_argument('--geofence', type=float, default=1.5)
    ap.add_argument('--config', help='read drone ids/positions/adjacency from a testbed yaml')
    ap.add_argument('--target-radius', type=float, default=None,
                    help='solve for starting positions giving this envelope (m)')
    ap.add_argument('--sweep', action='store_true',
                    help='list every viable theta instead of picking one')
    args = ap.parse_args()

    ids = None
    start = None
    lap = None
    if args.config:
        import yaml
        cfg = yaml.safe_load(open(args.config))
        drones = cfg['drones']
        ids = [d['id'] for d in drones]
        start = np.array([d.get('initial_position', [0.0, 0.0]) for d in drones])
        params = cfg.get('algorithm', {}).get('params', {})
        adjacency = params.get('adjacency')
        lap = (laplacian_from_adjacency(ids, adjacency) if adjacency
               else laplacian_cycle(len(ids)))
    else:
        ids = [f'drone{i + 1}' for i in range(args.drones)]
        lap = laplacian_cycle(args.drones)
        rng = np.random.default_rng(0)
        start = rng.uniform(-0.5, 0.5, (args.drones, 2))

    print(f"agents: {ids}")
    print(f"graph : {'from config' if args.config else 'directed cycle'}")
    eig_lap = sorted(np.linalg.eigvals(-lap), key=lambda z: -abs(z))
    print(f"eigenvalues of -L: {[f'{z:.3f}' for z in eig_lap]}")
    if len(ids) < 3:
        print("\nFewer than 3 agents: the coupling term cannot make a pattern.")
        return
    lam_k, lam_l = eig_lap[0], eig_lap[1]

    if args.sweep or args.theta is None:
        print("\nSearching theta for a configuration that is actually a trochoid...")
        viable = []
        for theta in np.linspace(0.05, math.pi - 0.05, 120):
            g = solve_gains(lam_k, lam_l, theta, args.beta)
            if not g:
                continue
            kappa, alpha, w1, w2 = g
            if kappa <= 0 or alpha <= 0 or abs(w1 - w2) < 1e-3:
                continue
            on_axis, unstable, _ = verify(lap, alpha, beta := args.beta, kappa, theta)
            if len(on_axis) == 2 and not unstable:
                viable.append((theta, kappa, alpha, on_axis))
        if not viable:
            print("  none found -- try a different beta or graph.")
            return
        print(f"  {len(viable)} viable theta values; "
              f"range {viable[0][0]:.2f} to {viable[-1][0]:.2f} rad")
        if args.sweep:
            print(f"\n  {'theta':>6} {'kappa':>8} {'alpha':>8}   frequencies")
            for theta, kappa, alpha, on_axis in viable[::max(1, len(viable) // 12)]:
                print(f"  {theta:6.3f} {kappa:8.3f} {alpha:8.3f}   {on_axis}")
            return
        # Pick the middle one: furthest from the edges where it degenerates.
        theta, kappa, alpha, _ = viable[len(viable) // 2]
        print(f"  picked theta={theta:.4f} (middle of the viable range)")
    else:
        theta = args.theta
        g = solve_gains(lam_k, lam_l, theta, args.beta)
        if not g:
            print("no solution at that theta")
            return
        kappa, alpha, _, _ = g

    if args.target_radius:
        solved = place_agents(lap, alpha, args.beta, kappa, theta,
                              args.target_radius)
        if solved is None:
            print("\ncould not solve for placement at these gains")
            return
        start = solved
        print("\nSolved starting positions for a "
              f"{args.target_radius} m pattern:")
        for drone_id, xy in zip(ids, start):
            print(f"    {drone_id}: [{xy[0]:+.3f}, {xy[1]:+.3f}]")

    ok = report(lap, ids, alpha, args.beta, kappa, theta, start, args.geofence)
    print("\nPut these in the algorithm params block:\n")
    print(f"    alpha: {alpha:.4f}")
    print(f"    beta:  {args.beta:.4f}")
    print(f"    kappa: {kappa:.4f}")
    print(f"    theta: {theta:.4f}")
    if not ok:
        print("\n(envelope exceeds the geofence -- shrink initial_position values)")


if __name__ == '__main__':
    main()
