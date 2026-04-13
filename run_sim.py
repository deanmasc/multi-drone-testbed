"""Standalone simulation -- no ROS2 required.

Runs the full testbed simulation (physics + algorithm) and
displays a live matplotlib window.

Usage:
    python run_sim.py                          # uses config/testbed.yaml
    python run_sim.py --algo ConsensusFormation
    python run_sim.py --algo LeaderFollower
"""

import sys
import os
import argparse
import time
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Add the package to Python path so imports work without installing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ros2_ws', 'src', 'drone_testbed'))

import drone_testbed.algorithms  # triggers @register_algorithm decorators
from drone_testbed.algorithms.registry import get_algorithm, list_algorithms
from drone_testbed.dynamics.double_integrator import step as physics_step
from drone_testbed.utils.types import DroneState, ControlOutput
import yaml


# ── Config ────────────────────────────────────────────────────────────────────

CONFIG_PATH = os.path.join(
    os.path.dirname(__file__),
    'ros2_ws', 'src', 'drone_testbed', 'config', 'testbed.yaml'
)

DRONE_COLORS = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6',
    '#1abc9c', '#e67e22', '#34495e', '#e91e63', '#00bcd4',
]

TRAIL_LENGTH = 60


# ── Simulation state ───────────────────────────────────────────────────────────

class DroneSimState:
    def __init__(self, drone_id, initial_position, initial_velocity):
        self.drone_id = drone_id
        self.state = np.array([
            initial_position[0], initial_position[1],
            initial_velocity[0], initial_velocity[1],
        ])
        self.initial_state = self.state.copy()
        self.trail = deque(maxlen=TRAIL_LENGTH)

    @property
    def position(self):
        return self.state[0:2]

    @property
    def velocity(self):
        return self.state[2:4]

    def to_drone_state(self):
        return DroneState(
            drone_id=self.drone_id,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
        )

    def reset(self):
        self.state = self.initial_state.copy()
        self.trail.clear()


# ── Main sim loop ──────────────────────────────────────────────────────────────

def run(algo_name_override=None):
    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f)

    sim_cfg = config['simulation']
    dt = sim_cfg.get('dt', 0.1)
    bounds = sim_cfg.get('bounds', 2.0)
    control_every = max(1, round(
        (1.0 / sim_cfg.get('control_rate', 5.0)) / dt
    ))

    # Load drones
    drones = {}
    for d in config['drones']:
        drones[d['id']] = DroneSimState(
            d['id'],
            d.get('initial_position', [0.0, 0.0]),
            d.get('initial_velocity', [0.0, 0.0]),
        )
    drone_ids = list(drones.keys())

    # Load algorithm
    algo_cfg = config.get('algorithm', {})
    algo_name = algo_name_override or algo_cfg.get('name', 'LeaderFollower')
    algo_params = algo_cfg.get('params', {})

    algorithm = get_algorithm(algo_name)
    algorithm.configure(algo_params, drone_ids)
    print(f"Running algorithm: {algo_name}")
    print(f"Available algorithms: {list_algorithms()}")
    print("Press Ctrl+C to stop. Close the window to exit.")

    # Current control commands
    commands = {d: np.zeros(2) for d in drone_ids}

    # ── Matplotlib setup ───────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#16213e')

    ax.set_xlim(-bounds - 0.3, bounds + 0.3)
    ax.set_ylim(-bounds - 0.3, bounds + 0.3)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2, color='white')
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.tick_params(colors='white')

    arena = Circle((0, 0), bounds, fill=False, linestyle='--',
                   color='#4a90d9', linewidth=2, alpha=0.6)
    ax.add_patch(arena)

    plt.tight_layout()
    plt.ion()
    plt.show()

    step_count = 0
    sim_time = 0.0

    try:
        while plt.fignum_exists(fig.number):
            loop_start = time.time()

            # Physics step every drone
            for drone_id, drone in drones.items():
                drone.state = physics_step(drone.state, commands[drone_id], dt)

                # Clamp velocity
                speed = np.linalg.norm(drone.velocity)
                if speed > 0.7:
                    drone.state[2:4] *= 0.7 / speed

                # Clamp position to bounds
                drone.state[0:2] = np.clip(drone.state[0:2], -bounds, bounds)

                drone.trail.append(drone.position.copy())

            sim_time += dt
            step_count += 1

            # Algorithm update at control_rate
            if step_count % control_every == 0:
                states = {did: d.to_drone_state() for did, d in drones.items()}
                ctrl_outputs = algorithm.compute_controls(
                    states, control_every * dt
                )
                for did, ctrl in ctrl_outputs.items():
                    commands[did] = ctrl.acceleration.copy()

            # Redraw at ~20Hz
            if step_count % max(1, round(0.05 / dt)) == 0:
                ax.cla()
                ax.set_facecolor('#16213e')
                ax.set_xlim(-bounds - 0.3, bounds + 0.3)
                ax.set_ylim(-bounds - 0.3, bounds + 0.3)
                ax.set_aspect('equal')
                ax.grid(True, alpha=0.2, color='white')
                ax.set_xlabel('X (m)', color='white')
                ax.set_ylabel('Y (m)', color='white')
                ax.tick_params(colors='white')

                arena = Circle((0, 0), bounds, fill=False, linestyle='--',
                               color='#4a90d9', linewidth=2, alpha=0.6)
                ax.add_patch(arena)

                for i, (drone_id, drone) in enumerate(drones.items()):
                    color = DRONE_COLORS[i % len(DRONE_COLORS)]
                    pos = drone.position
                    vel = drone.velocity

                    # Trail
                    trail = list(drone.trail)
                    if len(trail) > 1:
                        trail_arr = np.array(trail)
                        for j in range(len(trail_arr) - 1):
                            alpha = 0.08 + 0.4 * (j / len(trail_arr))
                            ax.plot(
                                trail_arr[j:j+2, 0], trail_arr[j:j+2, 1],
                                color=color, alpha=alpha, linewidth=1.5,
                            )

                    # Drone marker
                    ax.plot(pos[0], pos[1], 'o', color=color,
                            markersize=14, markeredgecolor='white',
                            markeredgewidth=1.5, zorder=5)

                    # ID label
                    ax.annotate(
                        drone_id, (pos[0], pos[1]),
                        textcoords='offset points', xytext=(12, 8),
                        fontsize=9, color=color, fontweight='bold',
                    )

                    # Velocity arrow
                    speed = np.linalg.norm(vel)
                    if speed > 0.02:
                        ax.arrow(
                            pos[0], pos[1],
                            vel[0] * 0.4, vel[1] * 0.4,
                            head_width=0.06, head_length=0.04,
                            fc=color, ec=color, alpha=0.8, zorder=4,
                        )

                ax.set_title(
                    f'Algorithm: {algo_name}   |   t = {sim_time:.1f}s',
                    color='white', fontsize=13, pad=10,
                )
                fig.canvas.draw()
                fig.canvas.flush_events()

            # Pace the sim to roughly real-time
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--algo',
        default=None,
        help='Algorithm name (e.g. LeaderFollower, ConsensusFormation)',
    )
    args = parser.parse_args()
    run(algo_name_override=args.algo)
